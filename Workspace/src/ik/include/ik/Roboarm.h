#ifndef ROBOARM_HEADER
#define ROBOARM_HEADER


class Roboarm{
public:
    float shoulder_pos;
    float elbow_pos;
    float wrist_pos;
    const float shoulder_len;
    const float elbow_len;
    const float wrist_len;

    float shoulder_vel;
    float elbow_vel;
    float wrist_vel;

    void calc_velocities(float ee_x, float ee_y, float ee_phi);//updates joint velocities

    Roboarm(float l_s, float l_e, float l_w,float shoulder=0, float elbow=0, float wrist=0);

    ~Roboarm();


};
#endif
