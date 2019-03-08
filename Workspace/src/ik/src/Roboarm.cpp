#include <cmath>
#include <iostream>

#include "../include/ik/Roboarm.h"
#include <ros/ros.h>

using namespace std;

Roboarm::Roboarm(double lengths[3], double angles[3], double stepSize)
    : alpha(stepSize) {

  for (int i = 0; i < 3; i++) {
    linkVelocities[i] = 0;
    linkLengths[i] = lengths[i];
    linkAngles[i] = angles[i];
  }
}

bool Roboarm::calculateVelocities(double endEffector[3], double angles[3],
                                  bool limits) {

  // prevent elbow from fully extending (0 radian to shoulder link) or
  // determinant will be 0.
  if (abs(angles[1]) < 0.0872665) { // 5 degree tolerance
    cout << "Inverse jacobian unsolved. Halting motion";
    linkVelocities[0] = 0.0;
    linkVelocities[1] = 0.0;
    linkVelocities[2] = 0.0;
    return false;
  }

  double eeX = endEffector[0];
  double eeY = endEffector[1];
  double eePhi = endEffector[2];
  ROS_INFO("ex: %f, ey: %f, ePhi: %f", eeX, eeY, eePhi);

  linkAngles[0] = angles[0];
  linkAngles[1] = angles[1];
  linkAngles[2] = angles[2];
  ROS_INFO("link angles: %f, %f, %f", linkAngles[0], linkAngles[1],
           linkAngles[2]);

  double c1 = cos(linkAngles[0]);
  double s1 = sin(linkAngles[0]);

  double c12 = cos(linkAngles[0] + linkAngles[1]);
  double s12 = sin(linkAngles[0] + linkAngles[1]);

  double c123 = cos(linkAngles[0] + linkAngles[1] + linkAngles[2]);
  double s123 = sin(linkAngles[0] + linkAngles[1] + linkAngles[2]);

  double l1 = linkLengths[0];
  double l2 = linkLengths[1];
  double l3 = linkLengths[2];

  linkVelocities[0] = (l2 * c12 * eeX) + (l2 * s12 * eeY) +
                      (l2 * l3 * (c12 * s123 - s12 * c123) * eePhi);
  ROS_INFO("link vel: %f", linkVelocities[0]);

  linkVelocities[1] = (-l1 * c1 - l2 * c12) * eeX - (l1 * s1 + l2 * s12) * eeY -
                      (l1 * l3 * (c1 * s123 - s1 * c123) +
                       l2 * l3 * (c12 * s123 - s12 * c123)) *
                          eePhi;
  ROS_INFO("link vel: %f", linkVelocities[1]);

  linkVelocities[2] =
      (l1 * c1 * eeX) + (l1 * s1 * eeY) +
      (l1 * l2 * (c1 * s12 - s1 * c12) + l1 * l3 * (c1 * s123 - s1 * c123)) *
          eePhi;
  ROS_INFO("link vel: %f", linkVelocities[2]);

  //  halt if illegal motion attempted
  if (limits == true) {
    for (int i = 0; i < 3; i++) {
      if (abs(abs(linkAngles[i] + linkVelocities[i] * alpha) - PI) <
          0.174533) { // |(|theta|-pi)|<tolerance
        cout << "Joint " << i << " over-rotated " << endl;
        linkVelocities[0] = 0.0;
        linkVelocities[1] = 0.0;
        linkVelocities[2] = 0.0;
        return false;
      }
    }
  }
  return true;
}

double *Roboarm::calculatePose() {
  // calculate the anlges at the next time step
  double *nextAngles = new double[3];
  for (int i = 0; i < 3; i++) {
    nextAngles[i] = linkAngles[i] + linkVelocities[i] * alpha;
  }

  // shoulder base
  pose[0].x = 0.0;
  pose[0].y = 0.0;

  double angleSoFar = 0;
  for (int i = 1; i < 4; i++) {
    angleSoFar += nextAngles[i - 1];
    pose[i].x = pose[i - 1].x + linkLengths[i - 1] * cos(angleSoFar);
    pose[i].y = pose[i - 1].y + linkLengths[i - 1] * sin(angleSoFar);
  }

  return nextAngles;
}

Roboarm::~Roboarm() {}
