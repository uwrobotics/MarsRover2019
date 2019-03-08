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
  double linkAngles[3];

  Point pose[4]; // shoulder, elbox, wrist, ee
  // double eePhi;

  bool calculateVelocities(double endEffector[3], double angles[3],
                           bool limits = false);
  double *calculatePose();

  Roboarm(double lengths[3], double angles[3], double stepSize = 0.001);

  ~Roboarm();

private:
  double linkLengths[3];
  double alpha;
};

#endif
