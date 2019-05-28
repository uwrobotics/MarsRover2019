//
// Created by tom on 25/02/19.
//
#include <can_msgs/Frame.h>
#include <ros/ros.h>

double jointAngles[6] = {0.0, 60, -30, -20, 0, 0};
double jointVelocities[6] = {0.0};
double jointModes[4] = {0.0};

const int turntableIdx = 0;
const int shoulderIdx = 1;
const int elbowIdx = 2;
const int endEffectorIdx = 3;
const int wristPitchIdx = 3;
const int wristRollIdx = 4;
const int clawIdx = 5;

constexpr int modeCanIds[4] = {0x300, 0x302, 0x304, 0x400};
constexpr int ctrlCanIds[6] = {0x301, 0x303, 0x305, 0x401, 0x402, 0x404};

const u_int8_t modeOpenLoop = 0x00;
const u_int8_t modeIkVel = 0x01;
const u_int8_t modeIkPos = 0x02;

std::string joints[] = {"turntable", "shoulder", "elbow", "wrist_pitch", "wrist_roll", "claw"};

std::string canTopics[] = {
    "/can/arm_joints/turntable", "/can/arm_joints/shoulder",
    "/can/arm_joints/elbow",     "/can/arm_joints/wristpitch",
    "/can/arm_joints/wristroll", "/can/arm_joints/claw"};

ros::Publisher pubs[6];

void setJoint(int jointIdx, int jointMode, const void *dataPtr)
{
  double data = *(double *)(dataPtr);
  switch(jointMode)
  {
    case modeOpenLoop:
      jointAngles[jointIdx] += data * 0.1;
      ROS_INFO("Setting %s velocity %f", joints[jointIdx].c_str(), data);
      break;
    case modeIkVel:
      jointAngles[jointIdx] += data * 0.1;
      ROS_INFO("Setting %s velocity %f deg/s", joints[jointIdx].c_str(), data);
      break;
    case modeIkPos:
      jointAngles[jointIdx] = data;
      ROS_INFO("Setting %s position %f deg/s", joints[jointIdx].c_str(), data);
      break;
    default:
      break;
  }
  ROS_INFO("shoulder set to %f", jointVelocities[shoulderIdx]);
}

void canCmdCallback(can_msgs::FrameConstPtr frame) {
  ROS_INFO("received %x", frame->id);
  switch (frame->id) {
    case modeCanIds[turntableIdx]:
      jointModes[turntableIdx] = *(int *)(frame->data.data());
      break;
    case modeCanIds[shoulderIdx]:
      jointModes[shoulderIdx] = *(int *)(frame->data.data());
      break;
    case modeCanIds[elbowIdx]:
      jointModes[elbowIdx] = *(int *)(frame->data.data());
      break;
    case modeCanIds[endEffectorIdx]:
      jointModes[endEffectorIdx] = *(int *)(frame->data.data());
      break;
    case ctrlCanIds[turntableIdx]:
      setJoint(turntableIdx, jointModes[turntableIdx], frame->data.data());
      break;
    case ctrlCanIds[shoulderIdx]:
      setJoint(shoulderIdx, jointModes[shoulderIdx], frame->data.data());
      break;
    case ctrlCanIds[elbowIdx]:
      setJoint(elbowIdx, jointModes[elbowIdx], frame->data.data());
      break;
    case ctrlCanIds[wristPitchIdx]:
      setJoint(wristPitchIdx, jointModes[endEffectorIdx], frame->data.data());
      break;
    case ctrlCanIds[wristRollIdx]:
      setJoint(wristRollIdx, jointModes[endEffectorIdx], frame->data.data());
      break;
    case ctrlCanIds[clawIdx]:
      setJoint(clawIdx, jointModes[endEffectorIdx], frame->data.data());
      break;
    default:
      break;
  }
}

void PublishAngles() {
  can_msgs::Frame msg;
  msg.dlc = 8;
  for (int i = 0; i < 6; i++) {
    msg.id = 0x500 + i;
    *(double *)(msg.data.data()) = jointAngles[i];
    pubs[i].publish(msg);
  }
}

int main(int argc, char **argv) {
  // initializing the node
  ros::init(argc, argv, "arm_motor_sim");
  ros::NodeHandle nh;

  for (int i = 0; i < 6; i++) {
    pubs[i] = nh.advertise<can_msgs::Frame>(canTopics[i], 1);
  }
  ros::Subscriber sub = nh.subscribe("/CAN_transmitter", 12, canCmdCallback);

  ros::Rate loopRate(10);

  while (ros::ok()) {
    ros::spinOnce();
    // for (int i = 0; i < 6; i++) {
    //   jointAngles[i] += jointVelocities[i] * 0.1;
    // }
    PublishAngles();
    loopRate.sleep();
  }

  return 0;
}