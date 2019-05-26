//
// Created by tom on 20/05/18.
//

#include "autonomy/autonomy_master_logic.h"
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <console_message/console_message.h>
#include <string>
#include <std_srvs/SetBool.h>
#include <local_planner/SetGoal.h>
#include <local_planner/StatusRequest.h>
#include <console_message/console_message.h>
#include <console_message/console_msg.h>

CAutonomyMasterLogic::CAutonomyMasterLogic(ros::NodeHandle &nh)
    : m_state(eAutonomyState::IDLE), m_pGoal(nullptr),
      //m_pBacktrackGps(nullptr), m_pLocalPlannerStatus(nullptr),
      m_bSearchingForBall(false),
      m_bBallReached(false), m_bBallDetected(false), m_bBallLost(false) {

  /// Create Subscribers ///
  // Goal Subscriber
  std::string strGoalGpsTopic = "/goal/utm";
  ros::param::get("/autonomy/goal_gps_topic", strGoalGpsTopic);
  m_pGoalGpsSub = new ros::Subscriber(nh.subscribe(
      strGoalGpsTopic, 1, &CAutonomyMasterLogic::GoalGpsCallback, this));
  m_stopSub = nh.subscribe("/autonomy/stop", 1, &CAutonomyMasterLogic::StopMsgCallback, this);

//  // Local Planner Status Subscriber
//  std::string strLocalPlannerStatusTopic = "/local_planner/status";
//  ros::param::get("/autonomy/local_plan_status_topic",
//                  strLocalPlannerStatusTopic);
//  m_pLocalPlanStatusSub = new ros::Subscriber(
//      nh.subscribe(strLocalPlannerStatusTopic, 1,
//                   &CAutonomyMasterLogic::LocalPlannerStatusCallback, this));
//
//  // Ball Detection Subscriber
//  std::string strBallDetectionTopic = "/ball_tracker/detection";
//  ros::param::get("/autonomy/ball_detection_topic", strBallDetectionTopic);
//  m_pBallDetectionSub = new ros::Subscriber(
//      nh.subscribe(strBallDetectionTopic, 1,
//                   &CAutonomyMasterLogic::BallDetectionCallback, this));
//
//  // Ball follower Arrived Subscriber
//  std::string strBallFollowerArrivedTopic = "/ball_following/arrival";
//  ros::param::get("/autonomy/ball_follower_arrived_topic",
//                  strBallFollowerArrivedTopic);
//  m_pBallFollowerArrivedSub = new ros::Subscriber(
//      nh.subscribe(strBallFollowerArrivedTopic, 1,
//                   &CAutonomyMasterLogic::BallFollowerArrivedCallback, this));
//
//  // Ball follower Lost Subscriber
//  std::string strBallFollowerLostTopic = "/ball_tracker/lost_ball";
//  ros::param::get("/autonomy/ball_tracker_lost_topic",
//                  strBallFollowerLostTopic);
//  m_pBallFollowerLostSub = new ros::Subscriber(
//      nh.subscribe(strBallFollowerLostTopic, 1,
//                   &CAutonomyMasterLogic::BallFollowerLostCallback, this));

  /// Create Publishers ///
//  // Local Planner target Publisher
//  std::string strLocalPlannerTargetTopic = "/local_planner/goal_gps";
//  ros::param::get("/autonomy/target_gps_topic", strLocalPlannerTargetTopic);
//  m_pTargetGpsPub = new ros::Publisher(
//      nh.advertise<sensor_msgs::NavSatFix>(strLocalPlannerTargetTopic, 1));
//
//  // LocalPlanner Enable Publisher
//  std::string strLocalPlannerEnableTopic = "/local_planner/enable";
//  ros::param::get("/autonomy/local_planner_enable_topic",
//                  strLocalPlannerEnableTopic);
//  m_pLocalPlannerEnablePub = new ros::Publisher(
//      nh.advertise<std_msgs::Bool>(strLocalPlannerEnableTopic, 1));
//
//  // Spiral Enable Publisher
//  std::string strSpiralEnableTopic = "/spiral/enable";
//  ros::param::get("/autonomy/spiral_enable_topic", strSpiralEnableTopic);
//  m_pSpiralEnablePub =
//      new ros::Publisher(nh.advertise<std_msgs::Bool>(strSpiralEnableTopic, 1));
//
//  // BallTracker Enable Publisher
//  std::string strBallTrackerEnableTopic = "/ball_tracker/enable";
//  ros::param::get("/autonomy/ball_tracker_enable_topic",
//                  strBallTrackerEnableTopic);
//  m_pBallTrackerPub = new ros::Publisher(
//      nh.advertise<std_msgs::Bool>(strBallTrackerEnableTopic, 1));

  /// Service Clients ///
  m_localPlannerGoalClient = nh.serviceClient<local_planner::SetGoal>("/local_planner/set_goal");
  m_localPlannerStatusClient = nh.serviceClient<local_planner::StatusRequest>("/local_planner/request_status");
  m_localPlannerEnableClient = nh.serviceClient<std_srvs::SetBool>("/local_planner/set_enabled");
  m_tbTrackerClient = nh.serviceClient<std_srvs::SetBool>("/tennis_ball_tracker/set_enabled");
  m_tbFollowerClient = nh.serviceClient<std_srvs::SetBool>("/tennis_ball_follower/set_enabled");
  m_spiralClient = nh.serviceClient<std_srvs::SetBool>("/spiral/set_enabled");

  m_pTwistMux = new CAutonomyTwistMux(nh);
}

////////////////////////////
/// Subscriber Callbacks ///
////////////////////////////

void CAutonomyMasterLogic::GoalGpsCallback(
    geometry_msgs::Pose2DConstPtr pGoal) {
  m_pGoal = pGoal;
  if (m_state == eAutonomyState::LOCALPLAN) {
    SetLocalPlannerGoal(*m_pGoal);
  }
  ROS_INFO("received goal");
}

//void CAutonomyMasterLogic::BacktrackGpsCallback(
//    sensor_msgs::NavSatFixConstPtr pBacktrackGps) {
//  m_pBacktrackGps = pBacktrackGps;
//  if (m_state == eAutonomyState::BACKTRACK) {
//    m_pTargetGpsPub->publish(m_pBacktrackGps);
//  }
//}
//
//void CAutonomyMasterLogic::LocalPlannerStatusCallback(
//    local_planner::LocalPlannerStatusConstPtr pLocalPlannerStatus) {
//  m_pLocalPlannerStatus = pLocalPlannerStatus;
//}
//
//void CAutonomyMasterLogic::BallDetectionCallback(
//    ball_tracker::BallDetectionConstPtr pBallDetection) {
//  m_bBallDetected = (pBallDetection->isDetected && pBallDetection->isStable);
//}
//
//void CAutonomyMasterLogic::BallFollowerArrivedCallback(
//    std_msgs::EmptyConstPtr pMsg) {
//  m_bBallReached = true;
//}
//
//void CAutonomyMasterLogic::BallFollowerLostCallback(
//    std_msgs::EmptyConstPtr pMsg) {
//  m_bBallLost = true;
//}

///////////////
/// Helpers ///
///////////////

// void CAutonomyMasterLogic::UpdateState() {
//   switch (m_state) {
//   case eAutonomyState::LOCALPLAN:
//     if (m_bBallDetected) {
//       StateTransition(eAutonomyState::TENNISBALL_FOLLOW);
//     } else if (m_pBacktrackGps &&
//                TimeSinceMessage(m_pBacktrackGps->header.stamp) <
//                    2.0) // TODO: softcode time
//     {
//       StateTransition(eAutonomyState::BACKTRACK);
//     } else if (m_pLocalPlannerStatus && m_pLocalPlannerStatus->goalReached) {
//       StateTransition(eAutonomyState::TENNISBALL_SEARCH);
//     }
//     break;
//   case eAutonomyState::BACKTRACK:
//     if (m_bBallDetected) {
//       StateTransition(eAutonomyState::TENNISBALL_FOLLOW);
//     } else if (m_pBacktrackGps &&
//                TimeSinceMessage(m_pBacktrackGps->header.stamp) > 2.0) {
//       // switch back to local planner
//       StateTransition(eAutonomyState::LOCALPLAN);
//     } else if (m_pLocalPlannerStatus && m_pLocalPlannerStatus->goalReached) {
//       // reached old location but still no service -- what to do?
//     }
//     break;
//   case eAutonomyState::TENNISBALL_SEARCH:
//     if (m_bBallDetected) {
//       StateTransition(eAutonomyState::TENNISBALL_FOLLOW);
//     }
//     break;
//   case eAutonomyState::TENNISBALL_FOLLOW:
//     // TODO: if tennisballReached-->idle
//     if (m_bBallReached) {
//       StateTransition(eAutonomyState::IDLE);
//     } else if (m_bBallLost) {
//       StateTransition(eAutonomyState::TENNISBALL_SEARCH);
//     }
//     break;
//   case eAutonomyState::IDLE:
//     if (m_pGoalGps) {
//       StateTransition(eAutonomyState::LOCALPLAN);
//     }
//     break;
//   default:
//     // This shouldn't happen, maybe force it back to local planner?
//     break;
//   }
// }

void CAutonomyMasterLogic::StateTransition(eAutonomyState newState) {
  m_state = newState;
  switch (newState) {
  case eAutonomyState::LOCALPLAN:
    ConsoleMessage::SendMessage("Entering state: Local Planner");
    if (m_pGoal) {
      SetLocalPlannerGoal(*m_pGoal);
      EnableLocalPlanner(true);
      DisableTBFollow();
      DisableTBTracker();
      EnableSpiral(false);
      LEDSendStatus(eAutonomyLEDStatus::RUNNING);
    }
    break;
  case eAutonomyState::TENNISBALL_SEARCH:
    ConsoleMessage::SendMessage("Entering state: Ball Search");
    // TODO: start process, need spiral search interface
    {
      m_bBallDetected = false;
      m_bBallLost = false;
      m_bBallReached = false;

      m_bSearchingForBall = true;
      EnableLocalPlanner(false);
      EnableAndCheckTBTracker();
      EnableSpiral(true);
      DisableTBFollow();
      ConsoleMessage::SendMessage("Enabled ball tracker and spiral");
    }
    break;
  case eAutonomyState::TENNISBALL_FOLLOW:
    ConsoleMessage::SendMessage("Entering state: Ball Follow");
    // TODO: not sure if this is the right spot
    {
      m_bBallLost = false;
      m_bBallReached = false;

      m_bSearchingForBall = true;
      EnableAndCheckTBTracker();
      EnableAndCheckTBFollow();
      EnableSpiral(false);
      EnableLocalPlanner(false);
      ConsoleMessage::SendMessage("Enabled ball tracker and follower");
    }
    break;
  case eAutonomyState::IDLE: {
    ConsoleMessage::SendMessage("Entering state: Idle");
    m_pGoal= nullptr;
    m_bBallDetected = false;
    m_bBallReached = false;
    m_bBallLost = false;
    // TODO: endProcess
    m_bSearchingForBall = false;
    DisableTBFollow();
    DisableTBTracker();
    EnableSpiral(false);
    EnableLocalPlanner(false);
    ConsoleMessage::SendMessage("Disabled all components");
  } break;
  default:
    // This shouldn't happen, maybe force it back to local planner?
    break;
  }
}

// Do actions that should be done every iteration for the given state
void CAutonomyMasterLogic::RunState() {
  switch (m_state) {
  case eAutonomyState::LOCALPLAN:
  {
    // need to check if we're in range to start detecting
    local_planner::StatusRequestRequest req;
    local_planner::StatusRequestResponse resp;
    
    if (!m_localPlannerStatusClient.call(req, resp)) {
      ConsoleMessage::SendMessage("Failed to request status from local planner", ConsoleMessage::ERROR);
      break;
    }
    if (resp.status.goalInRange) {
      if (!m_bSearchingForBall) {
        // start looking for the ball since we might pass it
        EnableAndCheckTBTracker();
        m_bSearchingForBall = true;
        ConsoleMessage::SendMessage("In range -- starting search for ball");
      } else {
        // check the status of the ball searcher
        if (EnableAndCheckTBTracker()) {
          StateTransition(eAutonomyState::TENNISBALL_FOLLOW);
          break;
        }
      }
    }
    if (resp.status.goalReached) {
      ConsoleMessage::SendMessage("GPS target reached");
      StateTransition(eAutonomyState::TENNISBALL_SEARCH);
    }
    break;
  }
  case eAutonomyState::TENNISBALL_SEARCH:
  {
    if (EnableAndCheckTBTracker()) {
      ConsoleMessage::SendMessage("Ball detected - starting follow");
      StateTransition(eAutonomyState::TENNISBALL_SEARCH);
    }
    break;
  }
  case eAutonomyState::TENNISBALL_FOLLOW:
  {
    if (EnableAndCheckTBFollow()) {
      LEDSendStatus(eAutonomyLEDStatus::SUCCESS);
      ConsoleMessage::SendMessage("Goal Reached!!! Fuck yeah!!!");
      StateTransition(eAutonomyState::IDLE);
    } else if (!EnableAndCheckTBTracker()) {
      ConsoleMessage::SendMessage("Ball detection lost -- going back to search");
      StateTransition(eAutonomyState::TENNISBALL_SEARCH);
    }
    break;
  }
  case eAutonomyState::IDLE:
  {
    if (m_pGoal) {
      StateTransition(eAutonomyState::LOCALPLAN);
    }
    break;
  }
  default:
    // This shouldn't happen, maybe force it back to local planner?
    break;
  }
}


bool CAutonomyMasterLogic::SetLocalPlannerGoal(geometry_msgs::Pose2D goal) {
  local_planner::SetGoalRequest goal_request;
  local_planner::SetGoalResponse goal_response;
  goal_request.goal = *m_pGoal;
  if (!m_localPlannerGoalClient.call(goal_request, goal_response)) {
    ConsoleMessage::SendMessage("Failed to contact local planner", ConsoleMessage::ERROR);
    return false;
  } else {
    if (goal_response.ack) {
      std::stringstream msg;
      msg << "Successfully set Local Planner Goal to x: " << m_pGoal->x << ", y: " << m_pGoal->y; 
      ConsoleMessage::SendMessage(msg.str());
      return true;
    } else {
      ConsoleMessage::SendMessage("Failed to set local planner goal", ConsoleMessage::ERROR);
      return false;
    }
  }
}


bool CAutonomyMasterLogic::EnableAndCheckTBTracker() {
  std_srvs::SetBoolRequest req;
  std_srvs::SetBoolResponse resp;
  req.data = true;
  if (!m_tbTrackerClient.call(req, resp))
  {
    ConsoleMessage::SendMessage("Failed to enable Tennis Ball tracker", ConsoleMessage::ERROR);
    return false;
  }
  return resp.success;
}
bool CAutonomyMasterLogic::EnableAndCheckTBFollow() {
  std_srvs::SetBoolRequest req;
  std_srvs::SetBoolResponse resp;
  req.data = true;
  if (!m_tbFollowerClient.call(req, resp))
  {
    ConsoleMessage::SendMessage("Failed to enable Tennis Ball follower", ConsoleMessage::ERROR);
    return false;
  }
  return resp.success;
}
bool CAutonomyMasterLogic::EnableSpiral(bool bEnabled) {
  std_srvs::SetBoolRequest req;
  std_srvs::SetBoolResponse resp;
  req.data = bEnabled;
  if (!m_spiralClient.call(req, resp))
  {
    ConsoleMessage::SendMessage("Failed to constact spiral node", ConsoleMessage::ERROR);
    return false;
  }
  return true;
}
bool CAutonomyMasterLogic::DisableTBTracker() {
  std_srvs::SetBoolRequest req;
  std_srvs::SetBoolResponse resp;
  req.data = false;
  if (!m_tbTrackerClient.call(req, resp))
  {
    ConsoleMessage::SendMessage("Failed to disable Tennis Ball follower", ConsoleMessage::ERROR);
    return false;
  }
  return resp.success;
}
bool CAutonomyMasterLogic::DisableTBFollow() {
  std_srvs::SetBoolRequest req;
  std_srvs::SetBoolResponse resp;
  req.data = false;
  if (!m_tbFollowerClient.call(req, resp))
  {
    ConsoleMessage::SendMessage("Failed to disable Tennis Ball follower", ConsoleMessage::ERROR);
    return false;
  }
  return resp.success;
}
bool CAutonomyMasterLogic::EnableLocalPlanner(bool bEnabled) {
  std_srvs::SetBoolRequest req;
  std_srvs::SetBoolResponse resp;
  req.data = bEnabled;
  if (!m_localPlannerEnableClient.call(req, resp))
  {
    ConsoleMessage::SendMessage("Failed to contact local planner", ConsoleMessage::ERROR);
    return false;
  }
  return resp.success;
}

void CAutonomyMasterLogic::LEDSendStatus(eAutonomyLEDStatus status) {
  can_msgs::Frame frame;
  frame.dlc = 1;
  frame.id = 0x210;
  frame.data[0] = (uint8_t)status;
}

void CAutonomyMasterLogic::StopMsgCallback(std_msgs::EmptyConstPtr msg) {
  StateTransition(eAutonomyState::IDLE);
}

/////////////////
/// Main Loop ///
/////////////////

void CAutonomyMasterLogic::Start() {
  ros::Rate looprate(2);
  while (ros::ok()) {
    ros::spinOnce();
    //UpdateState();
    RunState();
    // twistmux
    m_pTwistMux->Arbitrate(m_state);
    looprate.sleep();
  }
}
