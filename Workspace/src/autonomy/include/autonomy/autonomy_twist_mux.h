//
// Created by tom on 21/05/18.
//

#ifndef PROJECT_AUTONOMY_TWIST_MUX_H
#define PROJECT_AUTONOMY_TWIST_MUX_H
#include "autonomy/autonomy_master.h"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

class CAutonomyTwistMux {
public:
  CAutonomyTwistMux(ros::NodeHandle &nh);
  void Arbitrate(eAutonomyState state);

private:
  /// Subscriber callbacks ///
  void LocalPlannerCallback(geometry_msgs::TwistConstPtr pVel);
  void EStopCallback(geometry_msgs::TwistConstPtr pVel);
  void SpiralCallback(geometry_msgs::TwistConstPtr pVel);
  void BallFollowCallback(geometry_msgs::TwistConstPtr pVel);

  /// Subscribers ///
  ros::Subscriber *m_pLocalPlannerSub;
  ros::Subscriber *m_pEStopSub;
  ros::Subscriber *m_pSpiralSub;
  ros::Subscriber *m_pBallFolowSub;

  /// Publishers ///
  ros::Publisher *m_pCmdVelPub;

  /// Received twists ///

  geometry_msgs::TwistConstPtr m_pLocalPlannerVel;
  geometry_msgs::TwistConstPtr m_pEStopVel;
  geometry_msgs::TwistConstPtr m_pSpiralVel;
  geometry_msgs::TwistConstPtr m_pBallFallowVel;
  geometry_msgs::Twist m_idleVel;

  ros::Time m_LocalPlanRecTime;
  ros::Time m_EStopRecTime;
  ros::Time m_SpiralRecTime;
  ros::Time m_BallFollowRecTime;
};

#endif // PROJECT_AUTONOMY_TWIST_MUX_H
