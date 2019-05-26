#ifndef ROBOARM_HEADER
#define ROBOARM_HEADER
#define PI 3.141592

struct Point {
  double x;
  double y;
};

class Roboarm {
public:
  double linkVelocities[3];

  Point pose[4]; // shoulder, elbox, wrist, ee
  // double eePhi;

  bool calculateVelocities(double endEffectorVel[3], double curAngles[3],
                           bool limits = false);
  bool calculatePositionIK(double endEffectorPos[3], double curAngles[3],
                           double anglesOut[3], bool limits = false);
  double *calculatePose();

  Roboarm(double lengths[3], double angles[3], double normTol = 0.1,
          double angleTol = 0.005, double stepSize = 0.05);

  ~Roboarm();

private:
  double linkLengths[3];
  double linkAngles[3];
  double alpha;

  double mNormTolerance2;
  double mAngleTolerance;
};

#endif
