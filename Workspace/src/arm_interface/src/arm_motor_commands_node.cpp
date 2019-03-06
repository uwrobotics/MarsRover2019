#include <arm_interface/ArmCmd.h>
#include <can_msgs/Frame.h>
#include <cmath>
#include <ik/Roboarm.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <stdbool.h>

// Desired pos
std::vector<double> desiredPos(6);
// Desired angles
std::vector<double> desiredAngles(6);
// Actual angles
std::vector<double> actualAngles(6);
// IK vels
std::vector<double> ikCmdVels(6);
// FK vels
std::vector<double> fkCmdVels(6);

bool ik_mode = false;

ros::Publisher desAnglesPub;
ros::Publisher desAnglesCanPub;
ros::Publisher actAnglesPub;
ros::Subscriber armCmdSub;
// ros::Subscriber canTSESub;
std::vector<ros::Subscriber> canSubs(6);

std::vector<bool> jointsReady(6);

std::string canTopics[] = {
    "/can/arm_joints/turntable", "/can/arm_joints/shoulder",
    "/can/arm_joints/elbow",     "/can/arm_joints/wristpitch",
    "/can/arm_joints/wristroll", "/can/arm_joints/claw"};

double link_lengths[] = {35.56, 40.64, 30.48};
double default_angles[] = {45.0 * M_PI / 180, -30.0 * M_PI / 180,
                           -15.0 * M_PI / 180};

Roboarm ikControl(link_lengths, default_angles);

size_t vel_ctrl_can_ids[] = {0x301, 0x302, 0x303, 0x401, 0x402, 0x403};
size_t pos_ctrl_can_ids[] = {0x309, 0x30A, 0x30B, 0x409, 0x40A, 0x40B};

void armCmdCallback(arm_interface::ArmCmdConstPtr msg) {
  ROS_INFO("received msg");
  ik_mode = msg->ik_status;
  ROS_INFO("ik status: %d", ik_mode);
  if (ik_mode) {
    ROS_INFO("received ik cmd");
    for (int i = 0; i < 6; i++) {
      ikCmdVels[i] = msg->data_points[i];
      ROS_INFO("%f", ikCmdVels[i]);
    }
  } else {
    for (int i = 0; i < 6; i++) {
      fkCmdVels[i] = msg->data_points[i];
    }
  }
}

void canCallback(can_msgs::FrameConstPtr msg) {
  size_t joint_id = msg->id - 0x500;
  jointsReady[joint_id] = true;

  actualAngles[joint_id] = *(double *)(msg->data.data());
}

void PublishAngles() {
  std_msgs::Float64MultiArray msg;
  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg.layout.dim[0].size = 6;
  msg.layout.dim[0].stride = 1;
  msg.layout.dim[0].label =
      "angles"; // or whatever name you typically use to index vec1
  msg.data.clear();
  msg.data.insert(msg.data.end(), desiredAngles.begin(), desiredAngles.end());
  desAnglesPub.publish(msg);

  msg.data.clear();
  msg.data.insert(msg.data.end(), actualAngles.begin(), actualAngles.end());
  actAnglesPub.publish(msg);
}

void InitializeMode(bool velMode) {
  can_msgs::Frame msg;
  msg.data[0] = 1;
  msg.dlc = 1;
  uint32_t modeOffset = (velMode) ? 0 : 8;
  msg.id = 0x300 + modeOffset;
  desAnglesCanPub.publish(msg);
  msg.id = 0x400 + modeOffset;
  desAnglesCanPub.publish(msg);
}

void SendVelCommand() {
  if (ik_mode) {
    std::vector<double> cmdVels(6);
    cmdVels[0] = ikCmdVels[0];
    cmdVels[4] = ikCmdVels[4];
    cmdVels[5] = ikCmdVels[5];

    // TODO ik integration
    double endEffectorSpeed[] = {-ikCmdVels[1], -ikCmdVels[2],
                                 -ikCmdVels[3] * M_PI / 180};
    double currentAngles[] = {actualAngles[1] * M_PI / 180,
                              actualAngles[2] * M_PI / 180,
                              actualAngles[3] * M_PI / 180};
    if (ikControl.calculateVelocities(endEffectorSpeed, currentAngles)) {
      ROS_INFO("ik succeeded");
      for (int i = 0; i < 3; i++) {
        ROS_INFO("joint: %f", ikControl.linkVelocities[i]);
        if (ikControl.linkVelocities[i] > 5.0 * M_PI / 180) {

          double div = ikControl.linkVelocities[i] / (5.0 * M_PI / 180);
          ikControl.linkVelocities[0] /= div;
          ikControl.linkVelocities[1] /= div;
          ikControl.linkVelocities[2] /= div;
        }
      }
    } else {
      ROS_INFO("ik failed");
    }

    cmdVels[1] = ikControl.linkVelocities[0] * 180 / M_PI;
    cmdVels[2] = ikControl.linkVelocities[1] * 180 / M_PI;
    cmdVels[3] = ikControl.linkVelocities[2] * 180 / M_PI;

    can_msgs::Frame canMsg;
    canMsg.dlc = 8;
    for (int i = 0; i < 6; i++) {
      canMsg.id = vel_ctrl_can_ids[i];
      *(double *)(canMsg.data.data()) = cmdVels[i];
      desAnglesCanPub.publish(canMsg);
    }

  } else {
    can_msgs::Frame canMsg;
    canMsg.dlc = 8;
    for (int i = 0; i < 6; i++) {
      canMsg.id = vel_ctrl_can_ids[i];
      *(double *)(canMsg.data.data()) = fkCmdVels[i];
      desAnglesCanPub.publish(canMsg);
    }
  }
}

const bool velMode = true;

int main(int argc, char **argv) {
  // initializing the node
  ros::init(argc, argv, "arm_motor_commands");
  ros::NodeHandle nh;

  desAnglesCanPub = nh.advertise<can_msgs::Frame>("/CAN_transmitter", 6);
  actAnglesPub = nh.advertise<std_msgs::Float64MultiArray>(
      "/arm_interface/actual_joint_angles", 1);
  desAnglesPub = nh.advertise<std_msgs::Float64MultiArray>(
      "/arm_interface/desired_joint_angles", 1);

  for (int i = 0; i < 6; i++) {
    canSubs[i] = nh.subscribe(canTopics[i], 1, canCallback);
  }
  armCmdSub = nh.subscribe("/arm_interface/arm_cmd", 1, armCmdCallback);

  InitializeMode(velMode);

  int freq = 5;
  ros::Rate rate(freq);

  // loop that publishes info until the node is shut down
  while (ros::ok()) {
    ros::spinOnce();
    SendVelCommand();
    PublishAngles();
    rate.sleep();
  }

  return 0;
}
