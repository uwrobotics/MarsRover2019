#include <arm_interface/ArmCmd.h>
#include <can_msgs/Frame.h>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <stdbool.h>

// std::vector<double> targetAngles(2);
double targetAngleTheta, targetAngleY;

ros::Publisher targetAnglesCanPub;
ros::Subscriber targetAngleSub;

size_t can_id_theta = 0x800;
size_t can_id_y = 0x801;

void targetAnglesCallback(std_msgs::Pose2DConstPtr& msg) {
  ROS_INFO("received msg");
  targetAnglesTheta = msg.theta;
  targetAnglesY = msg.y;
  ROS_INFO("%f, %f", targetAnglesTheta, targetAnglesY);
}

void CanInitialize() {
  can_msgs::Frame msg;
  msg.data[0] = 1;
  msg.dlc = 1;
  msg.id = 0x800;
  targetAnglesCanPub.publish(msg);
}

void CanPublish() {
  can_msgs::Frame canMsg;
  canMsg.dlc = 8;

  canMsg.id = can_ids_theta;
  *(double *)(canMsg.data.data()) = targetAngleTheta;
  targetAnglesCanPub.publish(canMsg);

  canMsg.id = can_ids_y;
  *(double *)(canMsg.data.data()) = targetAngleY;
  targetAnglesCanPub.publish(canMsg);

  // for (int i = 0; i < 6; i++) {
  //   canMsg.id = vel_ctrl_can_ids[i];
  //   *(double *)(canMsg.data.data()) = fkCmdVels[i];
  //   targetAnglesCanPub.publish(canMsg);
  // }
}

int main(int argc, char **argv) {
  // initializing the node
  ros::init(argc, argv, "gimbal_interface");
  ros::NodeHandle nh;

  targetAnglesCanPub = nh.advertise<can_msgs::Frame>("/CAN_transmitter", 2);

  targetAngleSub = nh.subscribe("/gimbal_interface/target_angles", 1, targetAnglesCallback);

  CanInitialize();

  int freq = 5;
  ros::Rate rate(freq);

  // loop that publishes info until the node is shut down
  while (ros::ok()) {
    ros::spinOnce();
    CanPublish();
    rate.sleep();
  }

  return 0;
}
