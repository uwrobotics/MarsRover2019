//
// Created by tom on 25/07/18.
//

#include "LocalPlanner.h"

#include "std_msgs/String.h"
#include <cmath>
#include <string>

CLocalPlanner::CLocalPlanner(ros::NodeHandle *pNh,
                             const RobotParams_t &robotParams)
    : m_pNh(pNh), m_pCostmapSub(nullptr), m_pGoalGpsSub(nullptr),
      m_pPoseSub(nullptr), m_pOdometrySub(nullptr), m_pVelPub(nullptr),
      m_robotParams(robotParams), m_bVelReceived(false), m_bVelocityReady(true),
      m_pVelPubThread(nullptr), m_bGoalReached(false), m_bGoalInRange(false),
      m_bEnabled(true) {

  // subscribe to the goal gps coord
  std::string goal_gps_topic = "/goal/utm";
  ros::param::get("/local_planner/goal_gps_topic", goal_gps_topic);
  m_pGoalGpsSub = new ros::Subscriber(m_pNh->subscribe(
      goal_gps_topic, 1, &CLocalPlanner::GoalGPSCallback, this));

  // subscribe to the pose
  std::string pose_topic = "/localization/pose_utm";
  ros::param::get("/local_planner/pose_topic", pose_topic);
  m_pPoseSub = new ros::Subscriber(
      m_pNh->subscribe(pose_topic, 1, &CLocalPlanner::CurPoseCallback, this));

  // subscribe to the current odometry
  std::string odometry_topic = "/odometry/rover_gps_odom";
  ros::param::get("/local_planner/odometry_topic", odometry_topic);
  m_pOdometrySub = new ros::Subscriber(m_pNh->subscribe(
      odometry_topic, 1, &CLocalPlanner::OdometryCallback, this));

  // subscribe to the enable message
  std::string enable_topic = "/local_planner/enable";
  ros::param::get("/local_planner/enable_topic", enable_topic);
  m_pEnableSub = new ros::Subscriber(
      m_pNh->subscribe(enable_topic, 1, &CLocalPlanner::EnableCallback, this));

  // velocity publisher
  std::string velocity_topic = "/local_planner_cmd_vel";
  ros::param::get("/local_planner/velocity_out_topic", velocity_topic);
  m_pVelPub = new ros::Publisher(
      m_pNh->advertise<geometry_msgs::Twist>(velocity_topic, 1));

  // status publisher
  std::string status_topic = "/local_planner/status";
  ros::param::get("/local_planner/status_topic", status_topic);
  m_pStatusPub = new ros::Publisher(
      m_pNh->advertise<local_planner::LocalPlannerStatus>(status_topic, 1));

  // goal distance thresholds
  m_goalReachedDistThresh = 1.0;
  m_goalSearchDistThresh = 10.0;
  ros::param::get("/local_planner/goal_reached_distance",
                  m_goalReachedDistThresh);
  ros::param::get("/local_planner/goal_in_ranage_distance",
                  m_goalSearchDistThresh);

  // thread to continuously publish the desired velocity
  m_pVelPubThread = new std::thread(&CLocalPlanner::VelocityPublisher, this);

  m_pPlanningAlgo = new CPlanningAlgoBase(&m_robotParams, m_pNh);
}

// Callback for when a new goal gps coord is received.
void CLocalPlanner::GoalGPSCallback(geometry_msgs::Pose2DConstPtr pGoalMsg) {
  m_pGoalUtm = std::move(pGoalMsg);

  m_bGoalReached = false;
  m_bGoalInRange = false;

  if (m_pCurPoseUtm) {
    std::unique_lock<std::mutex> lock(m_velMutex);
    m_distToGoal =
        (m_pCurPoseUtm->y - m_pGoalUtm->y) *
            (m_pCurPoseUtm->y - m_pGoalUtm->y) +
        (m_pCurPoseUtm->x - m_pGoalUtm->x) * (m_pCurPoseUtm->x - m_pGoalUtm->x);
    lock.unlock();
    double globOrient = atan2(-(m_pGoalUtm->x - m_pCurPoseUtm->x),
                              m_pGoalUtm->y - m_pCurPoseUtm->y);
    m_orientationToGoal = globOrient - m_pCurPoseUtm->theta;
  }

  ROS_INFO("New goal: x=%f, y=%f", m_pGoalUtm->x, m_pGoalUtm->y);
}

// Callback for when an enable command is received.
void CLocalPlanner::EnableCallback(std_msgs::BoolConstPtr pEnableMsg) {
  m_bEnabled = pEnableMsg->data;
  if (!m_bEnabled) {
    std::unique_lock<std::mutex> lock(m_velMutex);
    m_bVelocityReady = false;
    m_bGoalReached = false;
    m_bGoalInRange = false;
    lock.unlock();
  }
}
//
void CLocalPlanner::CurPoseCallback(geometry_msgs::Pose2DConstPtr pPoseMsg) {
  ROS_WARN("pose received");
  m_pCurPoseUtm = std::move(pPoseMsg);

  ROS_INFO("Current pos: x=%f, y=%f, heading=%f", m_pCurPoseUtm->x,
           m_pCurPoseUtm->y, m_pCurPoseUtm->theta);

  // calculate the current relative orientation of the goal
  if (m_pGoalUtm) {
    std::unique_lock<std::mutex> lock(m_velMutex);
    m_distToGoal =
        (m_pCurPoseUtm->y - m_pGoalUtm->y) *
            (m_pCurPoseUtm->y - m_pGoalUtm->y) +
        (m_pCurPoseUtm->x - m_pGoalUtm->x) * (m_pCurPoseUtm->x - m_pGoalUtm->x);
    lock.unlock();

    double globOrient = atan2((m_pGoalUtm->y - m_pCurPoseUtm->y),
                              m_pGoalUtm->x - m_pCurPoseUtm->x);

    m_orientationToGoal = globOrient - m_pCurPoseUtm->theta;
    if (m_orientationToGoal > M_PI) {
      m_orientationToGoal -= 2 * M_PI;
    } else if (m_orientationToGoal < -M_PI) {
      m_orientationToGoal += 2 * M_PI;
    }

    ROS_INFO("globalOrientToGoal: %f, heading: %f, orientToGoal: %f",
             globOrient, m_pCurPoseUtm->theta, m_orientationToGoal);
  }
}

// Odometry callback
// Update velocity data
void CLocalPlanner::OdometryCallback(nav_msgs::Odometry::ConstPtr odometry) {

  m_curVel = odometry->twist.twist;

  // note that we have received odometry, which is necessary to start planning
  m_bVelReceived = true;

  // Debug statements
  static int count = 0;
  if (count == 0) {
    ROS_INFO("Current velocity: v=%f, w=%f", m_curVel.linear.x,
             m_curVel.angular.z);
  }
  count = (count + 1) % 1;
}

// Velocity publisher thread: continually publish the desired velocity so the
// rover keeps moving
void CLocalPlanner::VelocityPublisher() {
  ros::Rate rate(10);
  while (ros::ok()) {
    local_planner::LocalPlannerStatus statusMsg;

    std::unique_lock<std::mutex> lock(m_velMutex);

    bool bGoalReached = m_bGoalReached;
    bool bGoalInRange = m_bGoalInRange;

    statusMsg.distanceToGoal = sqrt(m_distToGoal);

    if (m_bVelocityReady) {
      geometry_msgs::Twist vel = m_targetVel;
      lock.unlock();

      if (!bGoalReached) {
        vel = m_targetVel;
      } else {
        // if we've reached our goal, stop moving
        vel = geometry_msgs::Twist(); // zero it
        vel.linear.x = 0;
        vel.angular.z = 0;
      }

      m_pVelPub->publish(vel);
    } else {
      lock.unlock();
    }

    statusMsg.goalReached = bGoalReached;
    statusMsg.goalInRange = bGoalInRange;

    m_pStatusPub->publish(statusMsg);
    rate.sleep();
  }
}

void CLocalPlanner::UpdateVelocity() {
  // Make sure we have the information requiredd for planning
  if (!m_bEnabled || !m_bVelReceived || !m_pCurPoseUtm || !m_pGoalUtm) {
    ROS_INFO("not performing local planner: not ready");
    return;
  }

  // check if we've already reached the goal
  std::unique_lock<std::mutex> lock(m_velMutex);
  m_distToGoal =
      (m_pCurPoseUtm->y - m_pGoalUtm->y) * (m_pCurPoseUtm->y - m_pGoalUtm->y) +
      (m_pCurPoseUtm->x - m_pGoalUtm->x) * (m_pCurPoseUtm->x - m_pGoalUtm->x);
  if (m_distToGoal < m_goalSearchDistThresh * m_goalSearchDistThresh) {
    ROS_INFO("Goal in range, start searching");
    m_bGoalInRange = true;
  }
  if (m_distToGoal < m_goalReachedDistThresh * m_goalReachedDistThresh) {
    m_bGoalReached = true;
    ROS_INFO("Goal reached\n");
    lock.unlock();
    return;
  }
  lock.unlock();

  // Determine the best speed
  geometry_msgs::Twist chosenVel = m_pPlanningAlgo->CalculateBestVelocity(
      m_curVel, m_orientationToGoal, m_distToGoal);
  // store the selected velocity so the publisher thread can publish it
  ROS_INFO("chose v=%f, w=%f", chosenVel.linear.x, chosenVel.angular.z);
  lock.lock();
  m_targetVel = chosenVel;
  m_bVelocityReady = true;
  lock.unlock();
}

void CLocalPlanner::Exec() {
  ros::Rate loopRate(10);
  while (ros::ok()) {
    ros::spinOnce();
    UpdateVelocity();
    loopRate.sleep();
  }
}