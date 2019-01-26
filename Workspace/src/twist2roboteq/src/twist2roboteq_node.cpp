#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "twist2roboteq_node.h"
#include "rover_physical_param.h"

ros::Publisher left_front_pub;
ros::Publisher left_middle_pub;
ros::Publisher left_back_pub;
ros::Publisher right_front_pub;
ros::Publisher right_middle_pub;
ros::Publisher right_back_pub;


const float SCALE = 1000;  // max value for roboteq
// const float MAX_WHEEL_SPEED = (MAX_X + WHEEL_DISTANCE/2 * MAX_Z)/RADIUS;

double left = 0;
double right = 0;


void twistCallback(const geometry_msgs::Twist::ConstPtr& vel)
{	
	ROS_INFO("lin: %f ang: %f", vel->linear.x, vel->angular.z);
	
	// float left = 0, right = 0;
	// left = ((vel->linear.x / MAX_X) - (vel->angular.z / MAX_Z))*SCALE;
	// right = ((vel-> linear.x / MAX_X) + (vel->angular.z / MAX_Z))*SCALE;

	// left = left > SCALE ? SCALE : left;
	// right = right > SCALE ? SCALE : right;

	// if(vel->linear.x < 0) {
	// 	float temp = left;
	// 	left = right;
	// 	right = temp;
	// }
	//left = ((vel->linear.x - WHEEL_DISTANCE/2 * vel->angular.z)/RADIUS) / MAX_WHEEL_SPEED * SCALE;
	//right = ((vel->linear.x + WHEEL_DISTANCE/2 * vel->angular.z)/RADIUS) / MAX_WHEEL_SPEED * SCALE;

    double left_vel = (vel->linear.x - vel->angular.z * WHEEL_DISTANCE / 2.0);
    double right_vel = (vel->linear.x + vel->angular.z * WHEEL_DISTANCE / 2.0);

    double left_rpm = (left_vel / RADIUS) * GEAR_RATIO * RPM_PER_RAD;
    double right_rpm = (right_vel / RADIUS) * GEAR_RATIO * RPM_PER_RAD;

    left = left_rpm / MAX_RPM * SCALE;
    right = right_rpm / MAX_RPM * SCALE;

    if (left > SCALE) {
        left = SCALE;
    }

    if (left < -SCALE) {
        left = -SCALE;
    }

    if (right > SCALE) {
        right = SCALE;
    }
    if (right < -SCALE) {
        right = -SCALE;
    }
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "twist2roboteq_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("cmd_vel", 100, twistCallback);
	left_front_pub = n.advertise<roboteq_msgs::Command&>("/left_front/cmd",1);
  left_middle_pub = n.advertise<roboteq_msgs::Command&>("/left_middle/cmd",1);
  left_back_pub = n.advertise<roboteq_msgs::Command&>("/left_back/cmd",1);
	right_front_pub = n.advertise<roboteq_msgs::Command&>("/right_front/cmd",1);
  right_middle_pub = n.advertise<roboteq_msgs::Command&>("/right_middle/cmd",1);
  right_back_pub = n.advertise<roboteq_msgs::Command&>("/right_back/cmd",1);

  double alpha = 0.95;

  roboteq_msgs::Command left_command;
  roboteq_msgs::Command right_command;

  ros::Rate loop_rate(60);
  while(ros::ok()) {
    loop_rate.sleep(); //Maintain the loop rate
    ros::spinOnce();   //Check for new messages


    left_command.mode = MODE_VELOCITY;
    right_command.mode = MODE_VELOCITY;

    left_command.setpoint = left * alpha + left_command.setpoint * (1 - alpha);
    right_command.setpoint = right * alpha + right_command.setpoint * (1 - alpha);

    left_front_pub.publish(left_command);
    left_middle_pub.publish(left_command);
    left_back_pub.publish(left_command);
    right_front_pub.publish(right_command);
    right_middle_pub.publish(right_command);
    right_back_pub.publish(right_command);
  }

  ROS_INFO("left: %f right: %f\r\n\r\n", left, right);

	ros::spin();
	return 0;
}
