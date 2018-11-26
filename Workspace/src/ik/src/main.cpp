#include "Roboarm.cpp"
#include <fstream>
#include <iostream>
main(){
    /*  //test basic output
    Roboarm arm(1.2, 0.9, 0.2, 1.0, 0.5, 0.2);
    arm.calc_velocities(1.4, 2.2, 0.0);
    std::cout <<arm.velocityLink1<< " "<<arm.velocityLink2<< " " <<arm.velocityLink3<<std::endl;
    arm.calc_pose();
    std::cout <<arm.eeX<< " "<<arm.eeY<< " " <<arm.eePhi<<std::endl;
    */

    Roboarm arm(1.7, 1.0, 0.3, 0.2, -1.5, -0.2);
    std::ofstream myfile;
    myfile.open("build/ik/ee_crds.csv");
    myfile << "eePhi,eeX,eeY,wristX,wristY,elbowX,elbowY,shoulderX,shoulderY,\n";

    int count = 0;
    while(count<2000){

        count++;
        //calculate joint velocities
        arm.calc_velocities(0.15,0.0, 0.000, arm.angleLink1, arm.angleLink2, arm.angleLink3); //move about 10cm/s

        //calculate EE pose 0.01s in the future and update joint positions
        arm.calc_pose(true);

        //save the coordinates of the EE
        myfile <<arm.eePhi<<","<<
                 arm.poseX[0] << "," << arm.poseY[0] << "," <<
                 arm.poseX[1] << "," << arm.poseY[1] << "," <<
                 arm.poseX[2] << "," << arm.poseY[2] << "," <<
                 arm.poseX[3] << "," << arm.poseY[3] << "\n";
        std::cout<< "saved "<<std::endl;

    }

    myfile.close();



}
