//
// Created by tom on 25/07/18.
//

#ifndef PROJECT_LOCALPLANNER_H
#define PROJECT_LOCALPLANNER_H

#include "PlanningAlgoBase.h"
#include "RoverParams.h"
#include "local_planner/LocalPlannerStatus.h"
//#include "occupancy_grid/OccupancyGrid.h"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/Point.h>
#include <mutex>
#include <thread>

class CLocalPlanner {
public:
  explicit CLocalPlanner(ros::NodeHandle *pNh,
                         const RobotParams_t &robotParams);
  // Main thread
  void Exec();

private:
  // Subscriber callbacks
  void GoalGPSCallback(geometry_msgs::Pose2DConstPtr pGoalMsg);
  void CurPoseCallback(geometry_msgs::Pose2DConstPtr pCurUtmMsg);
  // void OccupancyCallback(occupancy_grid::OccupancyGrid::ConstPtr grid);
  void OdometryCallback(nav_msgs::Odometry::ConstPtr odemetry);
  // void OdometryCallback(sensor_msgs::Imu::ConstPtr odometry);
  void EnableCallback(std_msgs::BoolConstPtr pEnableMsg);
  // publisher thread
  void VelocityPublisher();

  // Main update logic
  void UpdateVelocity();

  // Ros handlers
  ros::NodeHandle *m_pNh;
  ros::Subscriber *m_pCostmapSub;  // For costmap
  ros::Subscriber *m_pGoalGpsSub;  // For goal coord
  ros::Subscriber *m_pOdometrySub; // For velocity
  ros::Subscriber *m_pPoseSub;     // For current pose
  ros::Subscriber *m_pEnableSub;   // For enable command
  ros::Publisher *m_pVelPub;       // For publishing cmd_vel
  ros::Publisher *m_pStatusPub;    // For publishing status

  // status
  geometry_msgs::Twist m_curVel;               // Current velocity
  geometry_msgs::Pose2DConstPtr m_pCurPoseUtm; // Current pose
  geometry_msgs::Pose2DConstPtr m_pGoalUtm;    // Goal pose
  double m_orientationToGoal; // Counter clockwise angle to face the goal, in
                              // range [-pi, pi]

  // Parameters
  const RobotParams_t &m_robotParams;

  // Flags
  bool m_bVelReceived; // Whether current velocity is valid
  bool m_bGoalReached; // Whether rover is currently at its goal
  bool m_bGoalInRange; // Whether the goal is in range to start TB search
  double m_distToGoal; // Current distance left to the goal
  bool m_bEnabled;     // Whether the rover is currently enabled

  // velocity publishing
  std::mutex m_velMutex;
  geometry_msgs::Twist m_targetVel; // Chosen velocity
  bool m_bVelocityReady; // Whether a valid velocity has been calculated yet
  std::thread *m_pVelPubThread;

  double m_lastDwaCoordUtmX;
  double m_lastDwaCoordUtmY;

  // distance from goal thresholds
  double m_goalReachedDistThresh;
  double m_goalSearchDistThresh;

  // Planning Algorithm
  CPlanningAlgoBase *m_pPlanningAlgo;
};

#endif // PROJECT_LOCALPLANNER_H
