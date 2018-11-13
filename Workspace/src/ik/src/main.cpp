#include "Roboarm.cpp"
#include <iostream>
main(){
    Roboarm arm(1.5,2.3,3.5);
    std::cout<<arm.elbow_len<<" "<<arm.shoulder_pos<<std::endl;
    arm.calc_velocities(1.4,2.2,5.5);
    std::cout <<arm.shoulder_vel<< " "<<arm.elbow_vel<< " " <<arm.wrist_vel;

}
