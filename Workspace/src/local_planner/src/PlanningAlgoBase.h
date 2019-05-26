//
// Created by tom on 25/07/18.
//

#ifndef PROJECT_PLANNINGALGOBASE_H
#define PROJECT_PLANNINGALGOBASE_H

#include "RoverParams.h"

#include <geometry_msgs/Twist.h>
#include <ros/node_handle.h>

class CPlanningAlgoBase {
public:
  CPlanningAlgoBase(const RobotParams_t *robotParams, ros::NodeHandle *pNh);

  virtual geometry_msgs::Twist
  CalculateBestVelocity(const geometry_msgs::Twist &curVel,
                        double headingToGoal, double sqrDistToGoal);

protected:
  const RobotParams_t *m_pRobotParams;
};

#endif // PROJECT_PLANNINGALGOBASE_H
