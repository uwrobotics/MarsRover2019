//
// Created by tom on 25/02/19.
//
#include <ros/ros.h>
#include <can_msgs/Frame.h>


double jointAngles[6] = {0.0, 60, -30, -20, 0, 0};
double jointVelocities[6] = {0.0};

std::string canTopics[] = {"/can/arm_joints/turntable",
                           "/can/arm_joints/shoulder",
                           "/can/arm_joints/elbow",
                           "/can/arm_joints/wristpitch",
                           "/can/arm_joints/wristroll",
                           "/can/arm_joints/claw"};

ros::Publisher pubs[6];

void canCmdCallback(can_msgs::FrameConstPtr frame)
{
  ROS_INFO("received %x", frame->id);
  switch (frame->id)
  {
    case 0x301:
      jointVelocities[0] = *(double*)(frame->data.data());
      ROS_INFO("turntable set to %f", jointVelocities[0]);
      break;
    case 0x302:
      jointVelocities[1] = *(double*)(frame->data.data());
      ROS_INFO("shoulder set to %f", jointVelocities[1]);
      break;
    case 0x303:
      jointVelocities[2] = *(double*)(frame->data.data());
      ROS_INFO("elbow set to %f", jointVelocities[2]);
      break;
    case 0x401:
      jointVelocities[3] = *(double*)(frame->data.data());
      ROS_INFO("wristpitch set to %f", jointVelocities[3]);
      break;
    case 0x402:
      jointVelocities[4] = *(double*)(frame->data.data());
      ROS_INFO("wristroll set to %f", jointVelocities[4]);
      break;
    case 0x403:
      jointVelocities[5] = *(double*)(frame->data.data());
      ROS_INFO("claw set to %f", jointVelocities[5]);
      break;
    default:
      break;
  }
}

void PublishAngles()
{
  can_msgs::Frame msg;
  msg.dlc = 8;
  for(int i = 0; i < 6; i++)
  {
    msg.id = 0x500 + i;
    *(double*)(msg.data.data()) = jointAngles[i];
    pubs[i].publish(msg);
  }
}

int main(int argc, char** argv) {
  //initializing the node
  ros::init(argc, argv, "arm_motor_sim");
  ros::NodeHandle nh;

  for(int i = 0; i < 6; i++)
  {
    pubs[i] = nh.advertise<can_msgs::Frame>(canTopics[i], 1);
  }
  ros::Subscriber sub = nh.subscribe("/CAN_transmitter", 12, canCmdCallback);

  ros::Rate loopRate(10);

  while(ros::ok())
  {
    ros::spinOnce();
    for(int i = 0; i < 6; i++)
    {
      jointAngles[i] += jointVelocities[i] * 0.1;
    }
    PublishAngles();
    loopRate.sleep();
  }

  return 0;
}