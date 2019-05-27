//
// Created by tom on 29/05/18.
//

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>


double startW = 1.0;
double startV = 0.0;
double maxV = 0.5;
double wDecay = 0.995;
double vDecay = 0.995;
double minW = 0.1;


bool bEnabled = false;

geometry_msgs::Twist vel;

void InitializeVel()
{
  vel = geometry_msgs::Twist(); //reset to zero
  vel.linear.x = startV;
  vel.angular.z = startW;
  ROS_INFO("x: %f, w: %f", vel.linear.x, vel.angular.z);
}

bool SpiralEnableSrvCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &resp)
{
  bEnabled = req.data;
  if (!bEnabled)
  {
    InitializeVel();
  }
  resp.success = true;
  return true;
}

void UpdateVel()
{
  vel.angular.z = (vel.angular.z - minW) * wDecay + minW;
  vel.linear.x = maxV - (maxV - vel.linear.x) * vDecay;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "spiral_node");
  ros::NodeHandle nh;
  ROS_INFO("spiral");

  //Get params
  ros::param::get("/spiral/vStart", startV);
  ros::param::get("/spiral/wStart", startW);
  ros::param::get("/spiral/vDecay", vDecay);
  ros::param::get("/spiral/wDecay", wDecay);
  ros::param::get("/spiral/vMax", maxV);
  ros::param::get("/spiral/wMin", minW);
  ROS_INFO("startV %f, startW %f, vdecay %f, wDecay %f, maxV %f", startV, startW, vDecay, wDecay, maxV);
  InitializeVel();


  ros::ServiceServer enableSrv = nh.advertiseService("/spiral/set_enabled", SpiralEnableSrvCallback);
  ros::Publisher velPub = nh.advertise<geometry_msgs::Twist>("/spiral/cmd_vel",1);
  ros::Rate loopRate(10);
  ROS_INFO("spiral setup complete");

  while (ros::ok()) {
    ros::spinOnce();
    if (bEnabled)
    { 
      UpdateVel();
      velPub.publish(vel);
    }

    loopRate.sleep();
  }

}
