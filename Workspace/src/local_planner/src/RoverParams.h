//
// Created by tom on 25/07/18.
//

#ifndef PROJECT_ROVERPARAMS_H
#define PROJECT_ROVERPARAMS_H

typedef struct RobotParams {
  double maxV;
  double minV;
  double maxW;
  double maxLinAccel;
  double maxLinDecel;
  double maxAngAccel;
  double robotWidth;
  double robotLength;
  double timestep;

  double headingWeight;
  double distanceWeight;
  double velocityWeight;

} RobotParams_t;

#endif // PROJECT_ROVERPARAMS_H
