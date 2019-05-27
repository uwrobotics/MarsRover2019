//
// Created by tom on 25/07/18.
//

#include "PlanningAlgoBase.h"

static double clamp(double val, double minimum, double maximum) {
  return std::min(std::max(val, minimum), maximum);
}

CPlanningAlgoBase::CPlanningAlgoBase(const RobotParams_t *robotParams,
                                     ros::NodeHandle *pNh)
    : m_pRobotParams(robotParams) {}

geometry_msgs::Twist
CPlanningAlgoBase::CalculateBestVelocity(const geometry_msgs::Twist &curVel,
                                         double headingToGoal,
                                         double sqrDistToGoal) {
  geometry_msgs::Twist vel;
  if (fabs(headingToGoal) < M_PI_4) {
    vel.linear.x = std::min(m_pRobotParams->maxV / 2.0, sqrDistToGoal * m_pRobotParams->maxV/25);
  } else {
    vel.linear.x = 0.1;
  }
  vel.angular.z = clamp(m_pRobotParams->maxW * headingToGoal / M_PI_2, -m_pRobotParams->maxW, m_pRobotParams->maxW);
  return vel;
}
