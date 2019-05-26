//
// Created by tom on 20/05/18.
//

#include "autonomy/autonomy_master_logic.h"
#include <ros/ros.h>
#include <cstdio>
#include <console_message/console_message.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "autonomy_master");
  ros::NodeHandle nh;
  ROS_INFO("autonomy master logic");
  ConsoleMessage::Initialize(nh);

  CAutonomyMasterLogic masterLogic(nh);
  ROS_INFO("autonomy master logic setup complete");

  masterLogic.Start();

  return 0;
}