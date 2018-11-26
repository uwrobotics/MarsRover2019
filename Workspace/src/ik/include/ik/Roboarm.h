#ifndef ROBOARM_HEADER
#define ROBOARM_HEADER

struct Point{
    float x;
    float y;
    Point(const float &inX = 0.0, const float &inY = 0.0){
        x = inX;
        y = inY;
    }
};

class Roboarm {
  public:
    float velocityLink1;
    float velocityLink2;
    float velocityLink3;

    float angleLink1;
    float angleLink2;
    float angleLink3;

    Point pose [4];//shoulder, elbox, wrist, ee
    float eePhi;

    void calc_velocities(float endEffectorVX, float endEffectorVY, float endEffectorPhi,
                         float a1, float a2, float a3);
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
    

};



#endif
