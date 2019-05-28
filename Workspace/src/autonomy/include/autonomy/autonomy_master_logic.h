//
// Created by tom on 20/05/18.
//

#ifndef PROJECT_AUTONOMY_MASTER_LOGIC_H
#define PROJECT_AUTONOMY_MASTER_LOGIC_H

#include "autonomy/autonomy_master.h"
#include "autonomy/autonomy_twist_mux.h"
//#include "ball_tracker/BallDetection.h"
#include "local_planner/LocalPlannerStatus.h"
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <can_msgs/Frame.h>
#include <std_msgs/Empty.h>
//#include <sensor_msgs/NavSatFix.h>
//#include <std_msgs/Bool.h>
//#include <std_msgs/Empty.h>

class CAutonomyMasterLogic {
public:
  CAutonomyMasterLogic(ros::NodeHandle &nh);
  //~CAutonomyMasterLogic();
  void Start();

private:
  //void UpdateState();
  void StateTransition(eAutonomyState newState);
  void RunState();

  bool SetLocalPlannerGoal(geometry_msgs::Pose2D goal);
  bool EnableAndCheckTBTracker();
  bool EnableAndCheckTBFollow();
  bool EnableSpiral(bool bEnabled);
  bool DisableTBTracker();
  bool DisableTBFollow();
  bool EnableLocalPlanner(bool bEnabled);

  void LEDSendStatus(eAutonomyLEDStatus status);

  /// Subscriber Callbacks ///
  void GoalGpsCallback(geometry_msgs::Pose2DConstPtr pGoal);
  void StopMsgCallback(std_msgs::EmptyConstPtr msg);
  //void BacktrackGpsCallback(sensor_msgs::NavSatFixConstPtr pBacktrackGps);
  //void LocalPlannerStatusCallback(
  //    local_planner::LocalPlannerStatusConstPtr pLocalPlannerStatus); // --->SRV
  //void
  //BallDetectionCallback(ball_tracker::BallDetectionConstPtr pBallDetection); // --->SRV
  //void BallFollowerArrivedCallback(std_msgs::EmptyConstPtr pMsg); // --->SRV
  //void BallFollowerLostCallback(std_msgs::EmptyConstPtr pMsg); // --->SRV

  eAutonomyState m_state;
  bool m_bSearchingForBall;
  bool m_bBallDetected;
  bool m_bBallReached;
  bool m_bBallLost;

  /// Subscribers ///
  ros::Subscriber *m_pGoalGpsSub;
  ros::Subscriber m_stopSub;
  //ros::Subscriber *m_pBacktrackGpsSub;
  //ros::Subscriber *m_pLocalPlanStatusSub; // --->SRV
  //ros::Subscriber *m_pBallDetectionSub; // --->SRV
  //ros::Subscriber *m_pBallFollowerArrivedSub; // --->SRV
  //ros::Subscriber *m_pBallFollowerLostSub; // --->SRV

  /// Publishers ///
  ros::Publisher m_canPub;
  ros::Publisher m_successPub;
  //ros::Publisher *m_pTargetGpsPub;
  //ros::Publisher *m_pLocalPlannerEnablePub;
  // TODO: tennisball interface
  //ros::Publisher *m_pSpiralEnablePub;
  //ros::Publisher *m_pBallTrackerPub;

  /// Service Clients ///
  ros::ServiceClient m_localPlannerGoalClient;
  ros::ServiceClient m_localPlannerStatusClient;
  ros::ServiceClient m_localPlannerEnableClient;
  ros::ServiceClient m_tbTrackerClient;
  ros::ServiceClient m_tbFollowerClient;
  ros::ServiceClient m_spiralClient;

  /// Received messages ///
  geometry_msgs::Pose2DConstPtr m_pGoal;
  //sensor_msgs::NavSatFixConstPtr m_pBacktrackGps;
  //local_planner::LocalPlannerStatusConstPtr m_pLocalPlannerStatus;
  // TODO: tennis ball interface

  /// Twist Mux ///
  CAutonomyTwistMux *m_pTwistMux;
};

#endif // PROJECT_AUTONOMY_MASTER_LOGIC_H
