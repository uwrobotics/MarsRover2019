#ifndef ROBOARM_HEADER
#define ROBOARM_HEADER
#define PI 3.141592

struct Point{
    float x;
    float y;
};

class Roboarm {
  public:
    float linkVelocities[3];
    float linkAngles[3];

    Point pose [4]; // shoulder, elbox, wrist, ee
    float eePhi;

    void calculateVelocities(float endEffector[3], float angles[3]);
    float* calculatePose();

    Roboarm(float lengths[3], float angles[3], float stepSize = 0.001);
    
    ~Roboarm();

  private:
    float linkLengths[3];
    float alpha;
};

#endif
