#ifndef GUI_H
#define GUI_H
#define ODOMETRY_TOPIC "/odometry/rover_gps_odom" // "/imu"
#define TEST_TOPIC "chatter"
#define GPS_SENSOR_TOPIC                                                       \
  "/gps/filtered" // what does this mean? check to make sure this is right
#include <ros/network.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sstream>
#include <std_msgs/Int32.h> //need to include everything you are using form std_msgs!
#include <std_msgs/String.h>
#include <string>
#endif // GUI_H
