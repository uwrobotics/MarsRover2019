//
// Created by tom on 21/05/18.
//

#include "autonomy/autonomy_twist_mux.h"
#define EXPIRY_TIME (3.0)

CAutonomyTwistMux::CAutonomyTwistMux(ros::NodeHandle &nh)
    : m_pBallFallowVel(nullptr), m_pSpiralVel(nullptr),
      m_pLocalPlannerVel(nullptr), m_pEStopVel(nullptr) {

  // Create publisher
  m_pCmdVelPub =
      new ros::Publisher(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1));

  /// Create Subscribers ///
  // EStop Subscriber
  std::string strEStopTopic = "/estop/cmd_vel";
  ros::param::get("/autonomy/twist_mux/estop_vel_topic", strEStopTopic);
  m_pEStopSub = new ros::Subscriber(
      nh.subscribe(strEStopTopic, 1, &CAutonomyTwistMux::EStopCallback, this));

  // LocalPlanner Subscriber
  std::string strLocalPlannerTopic = "/local_planner/cmd_vel";
  ros::param::get("/autonomy/twist_mux/local_planner_vel_topic",
                  strLocalPlannerTopic);
  m_pLocalPlannerSub = new ros::Subscriber(nh.subscribe(
      strLocalPlannerTopic, 1, &CAutonomyTwistMux::LocalPlannerCallback, this));

  // Spiral Subscriber
  std::string strSpiralTopic = "/spiral/cmd_vel";
  ros::param::get("/autonomy/twist_mux/spiral_vel_topic", strSpiralTopic);
  m_pSpiralSub = new ros::Subscriber(nh.subscribe(
      strSpiralTopic, 1, &CAutonomyTwistMux::SpiralCallback, this));

  // BallFollower Subscriber
  std::string strBallFollowerTopic = "/tennis_ball_follower/cmd_vel";
  ros::param::get("/autonomy/twist_mux/ball_follow_vel_topic",
                  strBallFollowerTopic);
  m_pBallFolowSub = new ros::Subscriber(nh.subscribe(
      strBallFollowerTopic, 1, &CAutonomyTwistMux::BallFollowCallback, this));
}

void CAutonomyTwistMux::Arbitrate(eAutonomyState state) {
  geometry_msgs::Twist chosenVel;
  switch (state) {
  case eAutonomyState::LOCALPLAN:
    if (m_pEStopVel && (TimeSinceMessage(m_EStopRecTime) < EXPIRY_TIME)) {
      chosenVel = *m_pEStopVel;
    } else if (m_pLocalPlannerVel &&
               (TimeSinceMessage(m_LocalPlanRecTime) < EXPIRY_TIME)) {
      chosenVel = *m_pLocalPlannerVel;
    }
    break;
  case eAutonomyState::TENNISBALL_SEARCH:
    if (m_pEStopVel && (TimeSinceMessage(m_EStopRecTime) < EXPIRY_TIME)) {
      chosenVel = *m_pEStopVel;
    } else if (m_pSpiralVel &&
               (TimeSinceMessage(m_SpiralRecTime) < EXPIRY_TIME)) {
      chosenVel = *m_pSpiralVel;
    }
    break;
  case eAutonomyState::TENNISBALL_FOLLOW:
    if (m_pEStopVel && (TimeSinceMessage(m_EStopRecTime) < EXPIRY_TIME)) {
      chosenVel = *m_pEStopVel;
    } else if (m_pBallFallowVel &&
               (TimeSinceMessage(m_BallFollowRecTime) < EXPIRY_TIME)) {
      chosenVel = *m_pBallFallowVel;
    }
    break;
  case eAutonomyState::IDLE:
    if (m_pEStopVel && (TimeSinceMessage(m_EStopRecTime) < EXPIRY_TIME)) {
      chosenVel = *m_pEStopVel;
    } else {
      chosenVel = m_idleVel;
    }
    break;
  default:
    break;
  }
  m_pCmdVelPub->publish(chosenVel);
}

void CAutonomyTwistMux::LocalPlannerCallback(
    geometry_msgs::TwistConstPtr pVel) {
  m_pLocalPlannerVel = pVel;
  m_LocalPlanRecTime = ros::Time::now();
}

void CAutonomyTwistMux::EStopCallback(geometry_msgs::TwistConstPtr pVel) {
  m_pEStopVel = pVel;
  m_EStopRecTime = ros::Time::now();
}

void CAutonomyTwistMux::SpiralCallback(geometry_msgs::TwistConstPtr pVel) {
  m_pSpiralVel = pVel;
  m_SpiralRecTime = ros::Time::now();
}

void CAutonomyTwistMux::BallFollowCallback(geometry_msgs::TwistConstPtr pVel) {
  m_pBallFallowVel = pVel;
  m_BallFollowRecTime = ros::Time::now();
}
