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

    Roboarm arm(1.7, 1.0, 0.3, 0.0, 1.5, 0.0);
    std::ofstream myfile;
    myfile.open("src/ik/src/ee_crds.csv");
    myfile << "x_crd,y_crd,phi,\n";

    int count = 0;
    while(count<2500){

        count++;
        //calculate joint velocities
        arm.calc_velocities(0.10, 0.0, 0.00); //move about 10cm/s

        //calculate EE pose 0.01s in the future and update joint positions
        arm.calc_pose(true);

        //save the coordinates of the EE
        myfile << arm.eeX << "," << arm.eeY<<","<<arm.eePhi << "\n";
        std::cout<< "saved "<<std::endl;

    }

    myfile.close();



}
