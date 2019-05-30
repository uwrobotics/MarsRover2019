//
// Created by tom on 12/05/19.
//

#include <ros/ros.h>
#include <science_interface/science_status.h>
#include <can_msgs/Frame.h>
#include <rover_msgs/SetInt.h>
#include <rover_msgs/SetDouble.h>
#include <rover_msgs/SetFloat.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float32.h>

#define CAN_ELEVATOR_MODE_ID       0x200
#define CAN_AUGER_HEIGHT_ID        0x201
#define CAN_AUGER_SPEED_ID         0x202
#define CAN_CENTRIFUGE_SPINNING_ID 0x205
#define CAN_CENTRIFUGE_POS_ID      0x206
#define CAN_FUNNEL_OPEN_ID         0x207
#define CAN_SENSOR_MOUNT_DEPLOY_ID 0x208


enum elevatorMode {
  NONE,
  PWM,
  POS
};


class ScienceInterface {
public:
  ScienceInterface(ros::NodeHandle &nh);

  void spin()
  {
    int pubFreq = mSpinRate/mPublishingRate;
    ros::Rate loopRate(mSpinRate);
    int loopNum = 0;
    while (ros::ok())
    {
      loopRate.sleep();
      ros::spinOnce();
      if (loopNum % pubFreq == 0)
      {
        mScienceStatusPub.publish(mCurrentStatus);
      }
      loopNum++;
    }
  }

private:
  // CAN message callbacks
  void CAN_AugerHeight_Callback(can_msgs::FrameConstPtr frame)
  {
    mCurrentStatus.auger_height = *(float*)(&frame->data[0]);
  }
  void CAN_AugerSpeed_Callback(can_msgs::FrameConstPtr frame)
  {
    mCurrentStatus.auger_speed = *(float*)(&frame->data[0]);
  }
  void CAN_CentrifugeSpinning_Callback(can_msgs::FrameConstPtr frame)
  {
    mCurrentStatus.centrifuge_on = (bool)(frame->data[0]);
  }
  void CAN_CentrifugeSpeed_Callback(can_msgs::FrameConstPtr frame)
  {
    mCurrentStatus.centrifuge_speed = *(float*)(&frame->data[0]);
  }
  void CAN_CentrifugePos_Callback(can_msgs::FrameConstPtr frame)
  {
    mCurrentStatus.centrifuge_pos = *(int32_t*)(&frame->data[0]);
  }
  void CAN_FunnelStatus_Callback(can_msgs::FrameConstPtr frame)
  {
    mCurrentStatus.funnel_open = (bool)(frame->data[0]);
  }
  void CAN_SensorMountStatus_Callback(can_msgs::FrameConstPtr frame)
  {
    mCurrentStatus.sensors_mount_deployed = (bool)(frame->data[0]);
  }
  void CAN_Temperature_Callback(can_msgs::FrameConstPtr frame)
  {
    mCurrentStatus.temperature = *(float*)(&frame->data[0]);
  }
  void CAN_Moisture_Callback(can_msgs::FrameConstPtr frame)
  {
    mCurrentStatus.moisture = *(float*)(&frame->data[0]);
  }


  // GUI service calls callbacks
  bool SrvSetAugerHeight_Callback(rover_msgs::SetFloat::Request &req, rover_msgs::SetFloat::Response &resp)
  {
    if (mCurElevatorMode != elevatorMode::POS) {
      can_msgs::Frame modeFrame;
      modeFrame.dlc = 4;
      modeFrame.id = CAN_ELEVATOR_MODE_ID;
      //*(int *)(modeFrame.data.data()) = 1;
      modeFrame.data[0] = 1;
      mCANPub.publish(modeFrame);
      mCurElevatorMode = elevatorMode::POS;
    }

    can_msgs::Frame frame;
    frame.id = CAN_AUGER_HEIGHT_ID;
    frame.dlc = 4;
    *(float *)(frame.data.data()) = req.data;
    resp.data = req.data;
    mCANPub.publish(frame);
    return true;
  }
  bool SrvSetAugerSpeed_Callback(rover_msgs::SetFloat::Request &req, rover_msgs::SetFloat::Response &resp)
  {
    can_msgs::Frame frame;
    frame.id = CAN_AUGER_SPEED_ID;
    frame.dlc = 4;
    *(float *)(frame.data.data()) = req.data;
    mCANPub.publish(frame);
    return true;
  }
  bool SrvSetCentrifugeSpinning_Callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
  {
    can_msgs::Frame frame;
    frame.id = CAN_CENTRIFUGE_SPINNING_ID;
    frame.dlc = 4;
    *(int32_t *)(frame.data.data()) = (int32_t)req.data;
    mCANPub.publish(frame);
    return true;
  }
  bool SrvSetCentrifugePos_Callback(rover_msgs::SetInt::Request &req, rover_msgs::SetInt::Response &resp)
  {
    can_msgs::Frame frame;
    frame.id = CAN_CENTRIFUGE_POS_ID;
    frame.dlc = 4;
    *(int32_t *)(frame.data.data()) = req.data;
    mCANPub.publish(frame);
    return true;
  }
  bool SrvSetFunnelOpen_Callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
  {
    can_msgs::Frame frame;
    frame.id = CAN_FUNNEL_OPEN_ID;
    frame.dlc = 4;
    *(int32_t *)(frame.data.data()) = (int32_t)req.data;
    mCANPub.publish(frame);
    return true;
  }
  bool SrvSetSensorMountDeployed_Callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
  {
    can_msgs::Frame frame;
    frame.id = CAN_SENSOR_MOUNT_DEPLOY_ID;
    frame.dlc = 4;
    *(int32_t *)(frame.data.data()) = (int32_t)req.data;
    mCANPub.publish(frame);
    return true;
  }

  void ElevatorPWMCallback(std_msgs::Float32ConstPtr msg) {
    if (mCurElevatorMode != elevatorMode::PWM) {
      can_msgs::Frame modeFrame;
      modeFrame.dlc = 4;
      modeFrame.id = CAN_ELEVATOR_MODE_ID;
      //*(int *)(modeFrame.data.data()) = 0;
      modeFrame.data[0] = 0;
      mCANPub.publish(modeFrame);
      mCurElevatorMode = elevatorMode::PWM;
    }

    can_msgs::Frame frame;
    frame.id = CAN_AUGER_HEIGHT_ID;
    frame.dlc = 4;
    *(float *)(frame.data.data()) = msg->data;
    mCANPub.publish(frame);
  }

  science_interface::science_status mCurrentStatus;

  // CAN subscribers
  ros::Subscriber mCANAugerHeightSub;
  ros::Subscriber mCANAugerSpeedSub;
  ros::Subscriber mCANCentrifugeSpinningSub;
  ros::Subscriber mCANCentrifugeSpeedSub;
  ros::Subscriber mCANCentrifugePosSub;
  ros::Subscriber mCANFunnelSub;
  ros::Subscriber mCANSensorMountSub;
  ros::Subscriber mCANTemperatureSub;
  ros::Subscriber mCANMoistureSub;

  ros::Subscriber mElevatorPWMSub;

  // Services
  ros::ServiceServer mAugerHeightServer;
  ros::ServiceServer mAugerSpeedServer;
  ros::ServiceServer mCentrifugeOnServer;
  ros::ServiceServer mCentrifugePosServer;
  ros::ServiceServer mFunnelServer;
  ros::ServiceServer mSensorMountServer;

  // Status Data Publisher
  ros::Publisher mScienceStatusPub;

  // CAN Publisher
  ros::Publisher mCANPub;

  int mSpinRate;
  int mPublishingRate;

  elevatorMode mCurElevatorMode;

};

ScienceInterface::ScienceInterface(ros::NodeHandle &nh)
: mSpinRate(100), mPublishingRate(10), mCurElevatorMode(elevatorMode::NONE)
{
// Subscribe
  mCANAugerHeightSub = nh.subscribe("/can/science/auger_height", 1, &ScienceInterface::CAN_AugerHeight_Callback, this);
  mCANAugerSpeedSub = nh.subscribe("/can/science/auger_speed", 1, &ScienceInterface::CAN_AugerSpeed_Callback, this);
  mCANCentrifugeSpinningSub = nh.subscribe("/can/science/centrifuge_spinning", 1, &ScienceInterface::CAN_CentrifugeSpinning_Callback, this);
  mCANCentrifugeSpeedSub = nh.subscribe("/can/science/centrifuge_speed", 1, &ScienceInterface::CAN_CentrifugeSpeed_Callback, this);
  mCANCentrifugePosSub = nh.subscribe("/can/science/centrifuge_pos", 1, &ScienceInterface::CAN_CentrifugePos_Callback, this);
  mCANFunnelSub = nh.subscribe("/can/science/funnel", 1, &ScienceInterface::CAN_FunnelStatus_Callback, this);
  mCANSensorMountSub = nh.subscribe("/can/science/sensor_mount", 1, &ScienceInterface::CAN_SensorMountStatus_Callback, this);
  mCANTemperatureSub = nh.subscribe("/can/science/temperature", 1, &ScienceInterface::CAN_Temperature_Callback, this);
  mCANMoistureSub = nh.subscribe("/can/science/moisture", 1, &ScienceInterface::CAN_Moisture_Callback, this);

  mElevatorPWMSub = nh.subscribe("/science_interface/elevator_pwm", 1, &ScienceInterface::ElevatorPWMCallback, this);

  // Advertise
  mScienceStatusPub = nh.advertise<science_interface::science_status>("/science_interface/status", 1);
  mCANPub = nh.advertise<can_msgs::Frame>("/CAN_transmitter", 6);

  // Services
  mAugerHeightServer = nh.advertiseService("/science_interface/set_auger_height", &ScienceInterface::SrvSetAugerHeight_Callback, this);
  mAugerSpeedServer = nh.advertiseService("/science_interface/set_auger_speed", &ScienceInterface::SrvSetAugerSpeed_Callback, this);
  mCentrifugeOnServer = nh.advertiseService("/science_interface/set_centrifuge_spinning", &ScienceInterface::SrvSetCentrifugeSpinning_Callback, this);
  mCentrifugePosServer = nh.advertiseService("/science_interface/set_centrifuge_pos", &ScienceInterface::SrvSetCentrifugePos_Callback, this);
  mFunnelServer = nh.advertiseService("/science_interface/set_funnel_open", &ScienceInterface::SrvSetFunnelOpen_Callback, this);
  mSensorMountServer = nh.advertiseService("/science_interface/set_sensor_mount_deployed", &ScienceInterface::SrvSetSensorMountDeployed_Callback, this);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "science_interface_node");
  ros::NodeHandle nh;
  ScienceInterface scienceInterface(nh);
  scienceInterface.spin();
  return 0;
}


