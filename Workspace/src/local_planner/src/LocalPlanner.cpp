//
// Created by tom on 25/07/18.
//

#include "LocalPlanner.h"
//
// Created by tom on 20/02/18.
//

#include "LocalPlanner.h"
#include "robot_localization/navsat_conversions.h"
#include <cstdio>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
#include <cmath>
#include <robot_localization/navsat_conversions.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Int32MultiArray.h>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
//#include <visualization_msgs/Marker.h>

// Distance to remember obstacles near the corner for
#define IGNORE_DANGER_THRESHOLD 0.0 // 0.250

CLocalPlanner::CLocalPlanner(ros::NodeHandle *pNh,
                             const RobotParams_t &robotParams)
    : m_pNh(pNh), m_pOccupancySub(nullptr), m_pGoalGpsSub(nullptr),
    /*m_pCurGpsSub(nullptr), m_pVelSub(nullptr),*/ m_pVelPub(nullptr),
      m_robotParams(robotParams), m_bOdomReceived(false),
      m_bGoalReceived(false), m_bVelocityReady(true), m_pVelPubThread(nullptr),
    //m_distanceSinceLastRightDanger(1000), m_distanceSinceLastLeftDanger(1000),
      m_bGoalReached(false), m_bGoalInRange(false), m_bEnabled(true), m_bGpsReceived(false) {

  // subscribe to the occupancy grid
  //std::string occupancy_topic = "/OccupancyGrid";
  //ros::param::get("/local_planner/occupancy_topic", occupancy_topic);
  //m_pOccupancySub = new ros::Subscriber(m_pNh->subscribe(
  //        occupancy_topic, 1, &CLocalPlanner::OccupancyCallback, this));

  // subscribe to the goal gps coord
  std::string goal_gps_topic = "/local_planner/goal_gps";
  ros::param::get("/local_planner/goal_gps_topic", goal_gps_topic);
  m_pGoalGpsSub = new ros::Subscriber(m_pNh->subscribe(
      goal_gps_topic, 1, &CLocalPlanner::GoalGPSCallback, this));

//    m_pGpsSub = new ros::Subscriber(m_pNh->subscribe(
//            "/fix", 1, &CLocalPlanner::CurGPSCallback, this));

  // subscribe to the current odometry
  std::string odometry_topic = "/odometry/rover_gps_odom";
  ros::param::get("/local_planner/odometry_topic", odometry_topic);
  m_pOdometrySub = new ros::Subscriber(m_pNh->subscribe(
      odometry_topic, 1, &CLocalPlanner::OdometryCallback, this));

  // subscribe to the enable message
  std::string enable_topic = "/local_planner/enable";
  ros::param::get("/local_planner/enable_topic", enable_topic);
  m_pEnableSub = new ros::Subscriber(m_pNh->subscribe(
      enable_topic, 1, &CLocalPlanner::EnableCallback, this));

  // velocity publisher
  std::string velocity_topic = "/local_planner_cmd_vel";
  ros::param::get("/local_planner/velocity_out_topic", velocity_topic);
  m_pVelPub = new ros::Publisher(
      m_pNh->advertise<geometry_msgs::Twist>(velocity_topic, 1));

  // velocity publisher
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

  m_pPlanningAlgo = new CPlanningAlgoBase(&m_robotParams);
}

// Callback for when a new goal gps coord is received.
// Convert it to utm (TODO) and store
// void CLocalPlanner::GoalGPSCallback(sensor_msgs::NavSatFix::ConstPtr goal)
void CLocalPlanner::GoalGPSCallback(sensor_msgs::NavSatFixConstPtr goal) {
  //m_goalGPS = *goal;
  RobotLocalization::NavsatConversions::LLtoUTM(goal->latitude,
                                                goal->longitude,
                                                m_goalGpsUtmY,
                                                m_goalGpsUtmX,
                                                m_goalGpsUtmZone);

  // FOR SAR ONLY: use position in "map" frame
  //m_goalGpsUtmX = goal->x;
  //m_goalGpsUtmY = goal->y;

  m_bGoalReceived = true;
  m_bGoalReached = false;
  m_bGoalInRange = false;

  ROS_INFO("New goal: x=%f, y=%f", m_goalGpsUtmX, m_goalGpsUtmY);
}

// Callback for when a new goal gps coord is received.
// Convert it to utm (TODO) and store
// void CLocalPlanner::GoalGPSCallback(sensor_msgs::NavSatFix::ConstPtr goal)
void CLocalPlanner::EnableCallback(std_msgs::BoolConstPtr pEnableMsg) {
  m_bEnabled = pEnableMsg->data;
  if (!m_bEnabled) {
    std::unique_lock<std::mutex> lock(m_velMutex);
    m_bVelocityReady = false;
    m_bGoalReceived = false;
    m_bGoalReached = false;
    m_bGoalInRange = false;
    lock.unlock();
  }
}
//
//void CLocalPlanner::CurGPSCallback(sensor_msgs::NavSatFixConstPtr gps) {
//    ROS_WARN("received");
//    //m_goalGPS = *goal;
//    RobotLocalization::NavsatConversions::LLtoUTM(gps->latitude,
//                                                  gps->longitude,
//                                                  m_curGpsUtmY,
//                                                  m_curGpsUtmX,
//                                                  m_curGpsUtmZone);
//
//    // FOR SAR ONLY: use position in "map" frame
//    //m_goalGpsUtmX = goal->x;
//    //m_goalGpsUtmY = goal->y;
//
//    //m_bGoalReceived = true;
//    //m_bGoalReached = false;
//    //m_bGoalInRange = false;
//    m_bGpsReceived = true;
//    ROS_WARN("CurPos: x=%f, y=%f", m_curGpsUtmX, m_curGpsUtmY);
//}

// Odometry callback
// Convert the received location into UTM (TODO)
// and update heading information
void CLocalPlanner::OdometryCallback(nav_msgs::Odometry::ConstPtr odometry) {
//void CLocalPlanner::OdometryCallback(sensor_msgs::Imu::ConstPtr odometry) {
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped roverLocToUTM;
  try {
    roverLocToUTM = tfBuffer.lookupTransform(
        "utm",
        /*odometry->child_frame_id*/ odometry->child_frame_id /*header.frame_id*/, ros::Time(0),
        ros::Duration(2));
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
  }

  // ROS_INFO("Generated transform from base_link to utm");
  m_curGpsUtmX = roverLocToUTM.transform.translation.x;
  m_curGpsUtmY = roverLocToUTM.transform.translation.y;

  // FOR SAR ONLY: use position on map, figure out utm later
  // m_curGpsUtmX = odometry->pose.pose.position.x;
  // m_curGpsUtmY = odometry->pose.pose.position.y;

  m_curVel = odometry->twist.twist;

  // get the heading
  double heading = 0;
  tf::Quaternion q(
      odometry->pose.pose.orientation.x, odometry->pose.pose.orientation.y,
      odometry->pose.pose.orientation.z, odometry->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  heading = yaw;// - M_PI / 2;
  if (heading < -M_PI) {
    heading += 2 * M_PI;
  }

  // calculate the current relative orientation of the goal
  double globOrient =
      atan2(-(m_goalGpsUtmX - m_curGpsUtmX), m_goalGpsUtmY - m_curGpsUtmY);
  m_orientationToGoal = globOrient - heading;

  // note that we have received odometry, which is necessary to start planning
  m_bOdomReceived = true;

  // Debug statements
  static int count = 0;
  if (count == 0) {
    ROS_INFO("Current velocity: v=%f, w=%f", m_curVel.linear.x,
             m_curVel.angular.z);
    ROS_INFO("Current pos: x=%f, y=%f, heading=%f", m_curGpsUtmX, m_curGpsUtmY,
             heading);

    ROS_INFO("globalOrientToGoal: %f, heading: %f, orientToGoal: %f",
             globOrient, heading, m_orientationToGoal);
  }
  count = (count + 1) % 1;
}

/*
///Occupancy Callback:
1) construct dynamic window based on cur vel
2) assess distances
3) compute scores
4) select best
 */
//void CLocalPlanner::OccupancyCallback(
//        occupancy_grid::OccupancyGrid::ConstPtr grid) {
//    // Make sure we have the information requiredd for planning
//    if (!m_bEnabled || !m_bOdomReceived || !m_bGpsReceived || !m_bGoalReceived) {
//        ROS_WARN("ignoring occupancy: not ready");
//        return;
//    }
//
//    // check if we've already reached the goal
//    // TODO: publish a message for the global planner
//    std::unique_lock<std::mutex> lock(m_velMutex);
//    m_distToGoal =
//            (m_curGpsUtmY - m_goalGpsUtmY) * (m_curGpsUtmY - m_goalGpsUtmY) +
//            (m_curGpsUtmX - m_goalGpsUtmX) * (m_curGpsUtmX - m_goalGpsUtmX);
//    if (m_distToGoal < m_goalSearchDistThresh * m_goalSearchDistThresh) {
//        m_bGoalInRange = true;
//    }
//    if (m_distToGoal < m_goalReachedDistThresh * m_goalReachedDistThresh) {
//        m_bGoalReached = true;
//        ROS_INFO("Goal reached\n");
//        lock.unlock();
//        return;
//    }
//    lock.unlock();
//    ROS_INFO("Received an occupancy grid");
//
//    // Construct the dynamic window based on current velocity, and inform it if it
//    // needs
//    // to avoid an obstacle on one side
//    CDynamicWindow dynamicWindow(
//            m_curVel.linear.x, m_curVel.angular.z, m_robotParams,
//            (m_distanceSinceLastLeftDanger < IGNORE_DANGER_THRESHOLD),
//            (m_distanceSinceLastRightDanger < IGNORE_DANGER_THRESHOLD));
//    ROS_INFO("Created dynamic window");
//
//    // Determine the best speed
//    geometry_msgs::Twist chosenVel =
//            dynamicWindow.AssessOccupancyGrid(grid, m_orientationToGoal);
//
//    // Check for any danger conditions (obstacle near a from corner that may leave
//    // the cameras FOV
//    double distSinceLastCheck = sqrt((m_curGpsUtmX - m_lastDwaCoordUtmX) *
//                                     (m_curGpsUtmX - m_lastDwaCoordUtmX) +
//                                     (m_curGpsUtmY - m_lastDwaCoordUtmY) *
//                                     (m_curGpsUtmY - m_lastDwaCoordUtmY));
//    if (dynamicWindow.FoundDangerOnRight()) {
//        m_distanceSinceLastRightDanger = 0;
//        ROS_INFO("There was danger on right");
//    } else {
//        m_distanceSinceLastRightDanger += distSinceLastCheck;
//    }
//    if (dynamicWindow.FoundDangerOnLeft()) {
//        m_distanceSinceLastLeftDanger = 0;
//        ROS_INFO("there was danger on left");
//    } else {
//        m_distanceSinceLastLeftDanger += distSinceLastCheck;
//    }
//    ROS_INFO("Distance since danger: left: %f, right %f",
//             m_distanceSinceLastLeftDanger, m_distanceSinceLastRightDanger);
//    m_lastDwaCoordUtmX = m_curGpsUtmX;
//    m_lastDwaCoordUtmY = m_curGpsUtmY;
//
//    // store the selected velocity so the publisher thread can publish it
//    ROS_INFO("chose v=%f, w=%f", chosenVel.linear.x, chosenVel.angular.z);
//    lock.lock();
//    m_targetVel = chosenVel;
//    m_bVelocityReady = true;
//    lock.unlock();
//}

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
  if (!m_bEnabled || !m_bOdomReceived || !m_bGoalReceived) {
    //ROS_WARN("ignoring occupancy: not ready");
    return;
  }

  // check if we've already reached the goal
  // TODO: publish a message for the global planner
  std::unique_lock<std::mutex> lock(m_velMutex);
  m_distToGoal =
      (m_curGpsUtmY - m_goalGpsUtmY) * (m_curGpsUtmY - m_goalGpsUtmY) +
      (m_curGpsUtmX - m_goalGpsUtmX) * (m_curGpsUtmX - m_goalGpsUtmX);
  if (m_distToGoal < m_goalSearchDistThresh * m_goalSearchDistThresh) {
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
  geometry_msgs::Twist chosenVel =
      m_pPlanningAlgo->CalculateBestVelocity(m_curVel, m_orientationToGoal, m_distToGoal);
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