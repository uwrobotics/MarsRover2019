#include "Roboarm.h"
#include <cmath>
#include <iostream>

void calculate_fk(double linkLengths[3], double linkAngles[3],
                  double outPos[3]) {
  double c1 = cos(linkAngles[0]);
  double s1 = sin(linkAngles[0]);

  double c12 = cos(linkAngles[0] + linkAngles[1]);
  double s12 = sin(linkAngles[0] + linkAngles[1]);

  double c123 = cos(linkAngles[0] + linkAngles[1] + linkAngles[2]);
  double s123 = sin(linkAngles[0] + linkAngles[1] + linkAngles[2]);

  double l1 = linkLengths[0];
  double l2 = linkLengths[1];
  double l3 = linkLengths[2];

  outPos[0] = l1 * c1 + l2 * c12 + l3 * c123;
  outPos[1] = l1 * s1 + l2 * s12 + l3 * s123;
  outPos[2] = linkAngles[0] + linkAngles[1] + linkAngles[2];
}

int main() {
  double linkLengths[] = {35.56, 40.64, 30.48};
  double defaultAngles[] = {45.0 * M_PI / 180, -30.0 * M_PI / 180,
                            -15.0 * M_PI / 180};

  Roboarm ikControl(linkLengths, defaultAngles);

  // Do some tests on position ik control

  double trueJoints[2]
                   [3] = {{45 * M_PI / 180, -45 * M_PI / 180, 45 * M_PI / 180},
                          {60 * M_PI / 180, -10 * M_PI / 180, 8 * M_PI / 180}};
  double guessJoints[2][3] = {
      {40 * M_PI / 180, -42.3 * M_PI / 180, 48 * M_PI / 180},
      {59 * M_PI / 180, -8 * M_PI / 180, 10 * M_PI / 180}};

  for (int test_num = 0; test_num < 2; test_num++) {
    std::cout << "------------------------------------------------"
              << std::endl;
    double endEffector[3] = {0};
    calculate_fk(linkLengths, trueJoints[test_num], endEffector);

    double outJoints[3] = {0};
    std::cout << "Expected    : " << trueJoints[test_num][0] << ", "
              << trueJoints[test_num][1] << ", " << trueJoints[test_num][2]
              << std::endl;
    std::cout << "EndEffector : " << endEffector[0] << ", " << endEffector[1]
              << ", " << endEffector[2] << std::endl;
    std::cout << "Init guess  : " << guessJoints[test_num][0] << ", "
              << guessJoints[test_num][1] << ", " << guessJoints[test_num][2]
              << std::endl;
    ikControl.calculatePositionIK(endEffector, guessJoints[test_num],
                                  outJoints);

    std::cout << "Calculated  : " << outJoints[0] << ", " << outJoints[1]
              << ", " << outJoints[2] << std::endl;
    std::cout << "------------------------------------------------"
              << std::endl;
  }
  return 0;
}
