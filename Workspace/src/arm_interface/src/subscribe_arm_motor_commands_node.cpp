// This program subsribes to the CAN opic and recieves input from the Inverse Kinematics node and prints out the data on the screen
#include <ros/ros.h>

// A callback function definition that is executed each time a message of type InverseKInData is recieved from the CAN Topic which accepts a reference parameter to the message recieved

void messageRecieved(const InverseKinData& messageIR) {

// Not Sure what the 

}

int main(int argc, char** argv) {
  //initializing the node
  ros::init(argc, argv, "subscribe_arm_motor_commands");
  ros::NodeHandle nh;

  //Create a subscriber object
  ros::Subscriber sub = nh.subscribe("CAN", 10, &messageRecieved)

  ros::spin()

}
