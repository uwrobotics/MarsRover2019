//
// Created by tom on 20/05/18.
//

#ifndef PROJECT_AUTONOMY_MASTER_H
#define PROJECT_AUTONOMY_MASTER_H
#include <ros/ros.h>

typedef enum {
  LOCALPLAN,
  LOCALPLAN_TBSEARCH,
  TENNISBALL_SEARCH,
  TENNISBALL_FOLLOW,
  IDLE
} eAutonomyState;

typedef enum {
  ERROR = 0,
  RUNNING = 1,
  SUCCESS = 2
} eAutonomyLEDStatus;

static inline double TimeSinceMessage(const ros::Time &time) {
  return (ros::Time::now() - time).toSec();
}

#endif // PROJECT_AUTONOMY_MASTER_H
