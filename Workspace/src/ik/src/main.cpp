#include <iostream>
#include <fstream>
#include <string>

#include "Roboarm.cpp"
#define PI 3.141592

using namespace std;

int main(int argc, char **argv) {

	if (argc != 5) {
		cout << "Expected 4 arguments: filename, vx, vy, nSteps" << endl;
		return 1;
	}

	string fileName = argv[1];
	float vx = atof(argv[2]);
	float vy = atof(argv[3]);
	int nSteps = atoi(argv[4]);

	float lengths[3] = {0.4, 0.4, 0.4};
        float angles[3] = {-PI/4, PI/4, -PI/4};
	Roboarm arm(lengths, angles);

	std::ofstream myfile;

	string fileNameWithExtension = fileName + ".csv";
	myfile.open(fileNameWithExtension);

	if (!myfile.is_open()) {
		cout << "Couldn't open file";
		return -1;
	}

	float *currAngles = arm.calculatePose();
	for (int count = 0; count < nSteps; count++) {
		//calculate joint velocities
		float velocities[3] = {vx, vy, 0};

                arm.calculateVelocities(velocities, currAngles,true);
		currAngles = arm.calculatePose();

		myfile <<
			arm.pose[0].x << "," << arm.pose[0].y << " " <<
			arm.pose[1].x << "," << arm.pose[1].y << " " <<
			arm.pose[2].x << "," << arm.pose[2].y << " " <<
			arm.pose[3].x << "," << arm.pose[3].y << endl;
	}

	cout << "saved to " << fileNameWithExtension << endl;
	myfile.close();

	delete[] currAngles;
}
