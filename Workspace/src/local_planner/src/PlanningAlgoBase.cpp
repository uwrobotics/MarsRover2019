//
// Created by tom on 25/07/18.
//

#include "PlanningAlgoBase.h"

CPlanningAlgoBase::CPlanningAlgoBase(const RobotParams_t* robotParams)
: m_pRobotParams(robotParams) {

}

geometry_msgs::Twist
CPlanningAlgoBase::CalculateBestVelocity(const geometry_msgs::Twist &curVel, double headingToGoal, double distToGoal) {
  geometry_msgs::Twist vel;
  vel.linear.x = m_pRobotParams->maxV/2.0;
  vel.angular.z = m_pRobotParams->maxW * headingToGoal/M_PI;
  return vel;
}


