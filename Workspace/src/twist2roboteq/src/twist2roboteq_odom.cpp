#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h"
#include "roboteq_msgs/Feedback.h"
#include "rover_physical_param.h"

double enc_vel_left = 0;
double enc_vel_right = 0;

void left_feeback_callback(const roboteq_msgs::Feedback &msg) {
    enc_vel_left = msg.measured_velocity; // rad/s
}

void right_feedback_callback(const roboteq_msgs::Feedback &msg) {
    enc_vel_right = msg.measured_velocity; // rad/s
}


int main(int argc, char** argv){
  ros::init(argc, argv, "rover_encoder_odom");

  ros::NodeHandle n;

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("husky_velocity_controller/odom", 10);
  ros::Subscriber left_feeback_sub = n.subscribe("/left/feedback", 10, left_feeback_callback);
  ros::Subscriber right_feeback_sub = n.subscribe("/right/feedback", 10, right_feedback_callback);

  // integrated positions
  double pose_x = 0.0;
  double pose_y = 0.0;
  double pose_th = 0.0;

  // wheel velocities
  double v_left, v_right;

  // velocities
  double v_x, v_th; // pose_y velocity is always zero
  const double v_y = 0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate loop_rate(30);
  while(ros::ok()){

    loop_rate.sleep(); //Maintain the loop rate
    ros::spinOnce();   //Check for new messages

    current_time = ros::Time::now();

    v_left = enc_vel_left / GEAR_RATIO * RADIUS;
    v_right = enc_vel_right / GEAR_RATIO * RADIUS;

    v_x = (v_right + v_left) / 2;
    v_th = (v_right - v_left) / WHEEL_DISTANCE;

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (v_x * cos(pose_th) - v_y * sin(pose_th)) * dt;
    double delta_y = (v_x * sin(pose_th) + v_y * cos(pose_th)) * dt;
    double delta_th = v_th * dt;

    pose_x += delta_x;
    pose_y += delta_y;
    pose_th += delta_th;

    ROS_INFO("VEL: x=%f y=%f th=%f POS: x=%f y=%f th=%f", v_x, v_y, v_th*180/M_PI, pose_x, pose_y, pose_th*180/M_PI);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose_th);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = pose_x;
    odom.pose.pose.position.y = pose_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.pose.covariance = {
        1e-2, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 1e-2, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 1e-2, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 1e-2, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 1e-2, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 1e-2
    };

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = v_x;
    odom.twist.twist.linear.y = v_y;
    odom.twist.twist.angular.z = v_th;

    odom.twist.covariance = {
        1e-2, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 1e-2, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 1e-2, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 1e-2, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 1e-2, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 1e-2
    };

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
  }
}
