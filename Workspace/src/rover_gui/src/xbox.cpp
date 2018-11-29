// Code for xbox controller visualizer gui

#include <iostream>
	#include <ros/ros.h>
	
	#include <std_msgs/Float32.h>
	
	
	
	void SubCallback(std_msgs::Float32 msg)
	{
	std::cout << "Received message with data: " << msg.data << std::endl;
	}
	
	
		
	int main(int argc, char** argv)
	{
	//Initialize
	ros::init(argc, argv, "sub_node");
	ros::NodeHandle nh;
	
	//Create subscriber
	
	ros::Subscriber sub = nh.subscribe("/my_topic", 1, SubCallback);
	
	ros::spin();
	
	return 0;
	
	}
