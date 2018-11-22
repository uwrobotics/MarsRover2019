#include "Roboarm.cpp"
#include <iostream>
main(){
    Roboarm arm(1.2, 0.9, 0.2, 1.0, 0.5, 0.2);
    arm.calc_velocities(1.4, 2.2, 0.0);
    std::cout <<arm.velocityLink1<< " "<<arm.velocityLink2<< " " <<arm.velocityLink3<<std::endl;
    arm.calc_pose();
    std::cout <<arm.eeX<< " "<<arm.eeY<< " " <<arm.eePhi<<std::endl;

}
