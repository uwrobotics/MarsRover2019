#include <cmath>
#include <iostream>

#include "Roboarm.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>

using namespace std;
using namespace Eigen;

Roboarm::Roboarm(double lengths[3], double angles[3], double normTol,
                 double angleTol, double stepSize)
    : alpha(stepSize), mAngleTolerance(angleTol),
      mNormTolerance2(normTol * normTol) {

  for (int i = 0; i < 3; i++) {
    linkVelocities[i] = 0;
    linkLengths[i] = lengths[i];
    linkAngles[i] = angles[i];
  }
}

bool Roboarm::calculateVelocities(double endEffectorVel[3], double curAngles[3],
                                  bool limits) {

  // prevent elbow from fully extending (0 radian to shoulder link) or
  // determinant will be 0.
  if (abs(curAngles[1]) < 0.0872665) { // 5 degree tolerance
    cout << "Inverse jacobian unsolved. Halting motion";
    linkVelocities[0] = 0.0;
    linkVelocities[1] = 0.0;
    linkVelocities[2] = 0.0;
    return false;
  }

  double eeX = endEffectorVel[0];
  double eeY = endEffectorVel[1];
  double eePhi = endEffectorVel[2];
  ROS_INFO("ex: %f, ey: %f, ePhi: %f", eeX, eeY, eePhi);

  linkAngles[0] = curAngles[0];
  linkAngles[1] = curAngles[1];
  linkAngles[2] = curAngles[2];
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

  Matrix3d J;

  J << -(l1 * s1 + l2 * s12 + l3 * s123), -(l2 * s12 + l3 * s123), -l3 * s123,
      (l1 * c1 + l2 * c12 + l3 * c123), (l2 * c12 + l3 * c123), l3 * c123, 1, 1,
      1;

  Vector3d desVel;
  desVel << eeX, eeY, eePhi;

  Vector3d jointVel = J.colPivHouseholderQr().solve(desVel);

  if (!(J * jointVel).isApprox(desVel, 0.01)) {
    linkVelocities[0] = 0.0;
    linkVelocities[1] = 0.0;
    linkVelocities[2] = 0.0;
  } else {
    linkVelocities[0] = jointVel(0);
    linkVelocities[1] = jointVel(1);
    linkVelocities[2] = jointVel(2);
    ROS_INFO("link vel: %f", linkVelocities[0]);
    ROS_INFO("link vel: %f", linkVelocities[1]);
    ROS_INFO("link vel: %f", linkVelocities[2]);
  }

  /*
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
*/
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

// static Vector3d CalculateEEPose(Vector3d jointPos, double l[3])
//{
//
//}

bool Roboarm::calculatePositionIK(double endEffectorPos[3], double curAngles[3],
                                  double anglesOut[3],
                                  bool limits /*= false*/) {
  anglesOut[0] = 0;
  anglesOut[1] = 0;
  anglesOut[2] = 0;

  // prevent elbow from fully extending (0 radian to shoulder link) or
  // determinant will be 0.
  if (abs(curAngles[1]) < 0.0872665) { // 5 degree tolerance
    cout << "Inverse jacobian unsolved. Halting motion";
    return false;
  }

  double eeX = endEffectorPos[0];
  double eeY = endEffectorPos[1];
  double eePhi = endEffectorPos[2];
  // ROS_INFO("ex: %f, ey: %f, ePhi: %f", eeX, eeY, eePhi);

  // ROS_INFO("link angles: %f, %f, %f", curAngles[0], curAngles[1],
  //         curAngles[2]);

  double l1 = linkLengths[0];
  double l2 = linkLengths[1];
  double l3 = linkLengths[2];

  // Take an iterative approach to solve for the desired end position
  Vector3d desPos;
  desPos << eeX, eeY, eePhi;

  Vector3d curJoints;
  curJoints << curAngles[0], curAngles[1], curAngles[2];

  bool converged = false;
  for (int iteration = 0; iteration < 1000; iteration++) {
    // std::cout << "IT " << iteration << ": " << curJoints(0) << ", " <<
    // curJoints(1) << ", " << curJoints(2) << std::endl;

    double c1 = cos(curJoints(0));
    double s1 = sin(curJoints(0));

    double c12 = cos(curJoints(0) + curJoints(1));
    double s12 = sin(curJoints(0) + curJoints(1));

    double c123 = cos(curJoints(0) + curJoints(1) + curJoints(2));
    double s123 = sin(curJoints(0) + curJoints(1) + curJoints(2));

    Vector3d curPos;
    curPos << (l1 * c1 + l2 * c12 + l3 * c123),
        (l1 * s1 + l2 * s12 + l3 * s123), (curJoints.sum());

    Vector3d deltaPos = desPos - curPos;

    // std::cout << "Cur   pose: " << curPos << std::endl;
    // std::cout << "Delta pose: " << deltaPos << std::endl;
    // std::cout << "Mag2: " << deltaPos.squaredNorm() << std::endl;

    if (deltaPos.squaredNorm() < mNormTolerance2 &&
        abs(deltaPos(2)) < mAngleTolerance) {
      // std::cout << "converged" << std::endl;
      converged = true;
      break;
    }

    // Iteration: compute jacobian, solve approx joint change, update
    Matrix3d J;

    J << -(l1 * s1 + l2 * s12 + l3 * s123), -(l2 * s12 + l3 * s123), -l3 * s123,
        (l1 * c1 + l2 * c12 + l3 * c123), (l2 * c12 + l3 * c123), l3 * c123, 1,
        1, 1;

    Vector3d jointChange = J.colPivHouseholderQr().solve(deltaPos); // Slow
    // Vector3d jointChange = J.transpose() * deltaPos; // Faster, doesn't seem
    // to work as promised

    // std::cout << "Joint change: " << jointChange << std::endl;

    curJoints = curJoints + jointChange * alpha;
  }

  if (converged) {
    anglesOut[0] = curJoints(0);
    anglesOut[1] = curJoints(1);
    anglesOut[2] = curJoints(2);
  }

  return converged;
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
