#ifndef ROBOARM_HEADER
#define ROBOARM_HEADER

class Roboarm {
  public:
    float velocityLink1;
    float velocityLink2;
    float velocityLink3;

    float eeX;
    float eeY;
    float eePhi;

    void calc_velocities(float endEffectorVX, float endEffectorVY, float endEffectorPhi);
    void calc_pose(bool visualize = false);

    Roboarm(
      float l1, float l2, float l3,
      float a1 = 0, float a2 = 0, float a3 = 0,
      float stepSize = 0.001
      );
    
    ~Roboarm();

  private:
    const float alpha;

    const float lengthLink1;
    const float lengthLink2;
    const float lengthLink3;
    
    float angleLink1;
    float angleLink2;
    float angleLink3;
};

#endif
