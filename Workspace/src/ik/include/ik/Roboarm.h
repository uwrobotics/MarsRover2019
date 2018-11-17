#ifndef ROBOARM_HEADER
#define ROBOARM_HEADER

class Roboarm {
  public:
    void calc_velocities(float endEffectorVX, float endEffectorVY, float endEffectorPhi);

    Roboarm(
      float l1, float l2, float l3,
      float a1 = 0, float a2 = 0, float a3 = 0
      );
    
    ~Roboarm();
  private:
    const float lengthLink1;
    const float lengthLink2;
    const float lengthLink3;
    
    float angleLink1;
    float angleLink2;
    float angleLink3;

    float velocityLink1;
    float velocityLink2;
    float velocityLink3;
};

#endif
