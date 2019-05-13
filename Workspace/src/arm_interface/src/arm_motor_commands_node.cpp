#include <arm_interface/arm_mode.h>
#include <arm_interface/ArmCmd.h>
#include <can_msgs/Frame.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <stdbool.h>
#include "arm_interface.h"

std::string canTopics[] = {
    "/can/arm_joints/turntable", "/can/arm_joints/shoulder",
    "/can/arm_joints/elbow",     "/can/arm_joints/wristpitch",
    "/can/arm_joints/wristroll", "/can/arm_joints/claw"};

int main(int argc, char **argv) {
  // initializing the node
  ros::init(argc, argv, "arm_motor_commands");
  ros::NodeHandle nh;

  int freq = 5;
  float dT = 1.0/freq;

  ArmControlInterface armCtrlInterface(dT);

  ros::ServiceServer service = nh.advertiseService("/arm_interface/mode_service", &ArmControlInterface::SetMode, &armCtrlInterface);

  ros::Publisher desAnglesPub = nh.advertise<std_msgs::Float64MultiArray>("/arm_interface/desired_joint_angles", 1);
  ros::Publisher actAnglesPub = nh.advertise<std_msgs::Float64MultiArray>("/arm_interface/actual_joint_angles", 1);
  ros::Publisher canPub = nh.advertise<can_msgs::Frame>("/CAN_transmitter", 10);

  std::vector<ros::Subscriber> canSubs(6);
  for (int i = 0; i < 6; i++)
  {
    canSubs[i] = nh.subscribe(canTopics[i], 1, &ArmControlInterface::CanCallback, &armCtrlInterface);
  }

  ros::Subscriber armCmdSub = nh.subscribe("/arm_interface/arm_cmd", 1,  &ArmControlInterface::ArmCmdCallback, &armCtrlInterface);

  armCtrlInterface.SetPublishers(&desAnglesPub, &actAnglesPub, &canPub);

  ros::Rate rate(freq);

  // loop that publishes info until the node is shut down
  while (ros::ok()) {
    ros::spinOnce();

    armCtrlInterface.Run();

    rate.sleep();
  }

  return 0;

}
