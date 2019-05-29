//
// Created by tom on 25/07/18.
//

#include "LocalPlanner.h"

#include "std_msgs/String.h"
#include <cmath>
#include <string>
#include "DWAPlanner.h"

CLocalPlanner::CLocalPlanner(ros::NodeHandle *pNh,
                             const RobotParams_t &robotParams)
    : m_pNh(pNh), m_pCostmapSub(nullptr), m_pGoalGpsSub(nullptr),
      m_pPoseSub(nullptr), m_pOdometrySub(nullptr), m_pVelPub(nullptr),
      m_robotParams(robotParams), m_bVelReceived(false), m_bVelocityReady(true),
      m_pVelPubThread(nullptr), m_bGoalReached(false), m_bGoalInRange(false),
      m_bEnabled(true), m_bHasGoal(false) {

  // subscribe to the goal gps coord
  //std::string goal_gps_topic = "/goal/utm";
  //ros::param::get("/local_planner/goal_gps_topic", goal_gps_topic);
  //m_pGoalGpsSub = new ros::Subscriber(m_pNh->subscribe(
  //    goal_gps_topic, 1, &CLocalPlanner::GoalGPSCallback, this));

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
  //std::string enable_topic = "/local_planner/enable";
  //ros::param::get("/local_planner/enable_topic", enable_topic);
  //m_pEnableSub = new ros::Subscriber(
  //    m_pNh->subscribe(enable_topic, 1, &CLocalPlanner::EnableCallback, this));

  // services
  std::string goal_gps_service = "/local_planner/set_goal";
  ros::param::get("/local_planner/goal_gps_service", goal_gps_service);
  m_goalSrv = m_pNh->advertiseService(goal_gps_service, &CLocalPlanner::SetGoalSrvCallback, this);

  std::string status_req_service = "/local_planner/request_status";
  ros::param::get("/local_planner/status_req_service", status_req_service);
  m_statusReqSrv = m_pNh->advertiseService(status_req_service, &CLocalPlanner::StatusRequestSrvCallback, this);

  std::string set_enabled_service = "/local_planner/set_enabled";
  ros::param::get("/local_planner/set_enabled_service", set_enabled_service);
  m_setEnabledSrv = m_pNh->advertiseService(set_enabled_service, &CLocalPlanner::SetEnabledSrvCallback, this);



  // velocity publisher
  std::string velocity_topic = "/local_planner/cmd_vel";
  ros::param::get("/local_planner/velocity_out_topic", velocity_topic);
  m_pVelPub = new ros::Publisher(
      m_pNh->advertise<geometry_msgs::Twist>(velocity_topic, 1));

  // // status publisher
  // std::string status_topic = "/local_planner/status";
  // ros::param::get("/local_planner/status_topic", status_topic);
  // m_pStatusPub = new ros::Publisher(
  //     m_pNh->advertise<local_planner::LocalPlannerStatus>(status_topic, 1));

  // goal distance thresholds
  m_goalReachedDistThresh = 1.0;
  m_goalSearchDistThresh = 10.0;
  ros::param::get("/local_planner/goal_reached_distance",
                  m_goalReachedDistThresh);
  ros::param::get("/local_planner/goal_in_ranage_distance",
                  m_goalSearchDistThresh);

  // thread to continuously publish the desired velocity
  m_pVelPubThread = new std::thread(&CLocalPlanner::VelocityPublisher, this);

  std::string planner_type = "gps";
  ros::param::get("/local_planner/planner_type", planner_type);
  if (planner_type == "gps") {
    m_pPlanningAlgo = new CPlanningAlgoBase(&m_robotParams, m_pNh);
  } else {
    m_pPlanningAlgo = new DWAPlanner(&m_robotParams, m_pNh);
  }
}

// Callback for when a new goal gps coord is received.
bool CLocalPlanner::SetGoalSrvCallback(local_planner::SetGoalRequest& req, local_planner::SetGoalResponse& resp) {
  m_goalUtm = req.goal;
  m_bHasGoal = true;

  //std::unique_lock<std::mutex> lock(m_velMutex);
  m_bGoalReached = false;
  m_bGoalInRange = false;
  m_bVelocityReady = false;
  if (m_pCurPoseUtm) {
    m_distToGoal =
        (m_pCurPoseUtm->y - m_goalUtm.y) *
            (m_pCurPoseUtm->y - m_goalUtm.y) +
        (m_pCurPoseUtm->x - m_goalUtm.x) * (m_pCurPoseUtm->x - m_goalUtm.x);
  //   lock.unlock();
  //   double globOrient = atan2(-(m_goalUtm.x - m_pCurPoseUtm->x),
  //                             m_goalUtm.y - m_pCurPoseUtm->y);
  //   m_orientationToGoal = globOrient - m_pCurPoseUtm->theta;
  }
  //lock.unlock();
  ROS_INFO("New goal: x=%f, y=%f", m_goalUtm.x, m_goalUtm.y);
  resp.ack = true;
  return true;
}

// Callback for when an enable command is received.
bool CLocalPlanner::SetEnabledSrvCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &resp) {
  m_bEnabled = req.data;
  if (!m_bEnabled) {
    // std::unique_lock<std::mutex> lock(m_velMutex);
    m_bVelocityReady = false;
    m_bGoalReached = false;
    m_bGoalInRange = false;
    m_bHasGoal = false;
    // lock.unlock();
  }
  resp.success = true;
  return true;
}
//
void CLocalPlanner::CurPoseCallback(geometry_msgs::Pose2DConstPtr pPoseMsg) {
  ROS_WARN("pose received");
  m_pCurPoseUtm = std::move(pPoseMsg);

  ROS_INFO("Current pos: x=%f, y=%f, heading=%f", m_pCurPoseUtm->x,
           m_pCurPoseUtm->y, m_pCurPoseUtm->theta);

  // calculate the current relative orientation of the goal
  if (m_bHasGoal) {
    std::unique_lock<std::mutex> lock(m_velMutex);
    m_distToGoal =
        (m_pCurPoseUtm->y - m_goalUtm.y) *
            (m_pCurPoseUtm->y - m_goalUtm.y) +
        (m_pCurPoseUtm->x - m_goalUtm.x) * (m_pCurPoseUtm->x - m_goalUtm.x);
    lock.unlock();

    double globOrient = atan2((m_goalUtm.y - m_pCurPoseUtm->y),
                              m_goalUtm.x - m_pCurPoseUtm->x);

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

bool CLocalPlanner::StatusRequestSrvCallback(local_planner::StatusRequestRequest& req, local_planner::StatusRequestResponse& resp) {
  std::unique_lock<std::mutex> lock(m_velMutex);
  resp.status.distanceToGoal = sqrt(m_distToGoal);

  resp.status.goalReached = m_bGoalReached;
  resp.status.goalInRange = m_bGoalInRange;
  lock.unlock();
  return true;
}

// Velocity publisher thread: continually publish the desired velocity so the
// rover keeps moving
void CLocalPlanner::VelocityPublisher() {
  ros::Rate rate(10);
  while (ros::ok()) {
    std::unique_lock<std::mutex> lock(m_velMutex);

    bool bGoalReached = m_bGoalReached;

    if (m_bEnabled && m_bVelocityReady) {
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

    rate.sleep();
  }
}

void CLocalPlanner::UpdateVelocity() {
  // Make sure we have the information requiredd for planning
  if (!m_bEnabled || !m_bVelReceived || !m_pCurPoseUtm || !m_bHasGoal) {
    ROS_INFO("not performing local planner: not ready");
    return;
  }

  // check if we've already reached the goal
  std::unique_lock<std::mutex> lock(m_velMutex);
  m_distToGoal =
      (m_pCurPoseUtm->y - m_goalUtm.y) * (m_pCurPoseUtm->y - m_goalUtm.y) +
      (m_pCurPoseUtm->x - m_goalUtm.x) * (m_pCurPoseUtm->x - m_goalUtm.x);
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