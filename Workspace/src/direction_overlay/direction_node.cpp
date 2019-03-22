#include <iostream>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose2D.h>


using namespace cv;
using namespace std;

sensor_msgs::NavSatFixConstPtr pNavSatOrigin = nullptr;

void utmPubCallback(geometry_msgs::Pose2DConstPtr msg) { 
	curr = msg; 
}

void rgbImageCallback(const sensor_msgs::Image::ConstPtr& msg) {
	ROS_INFO("RGB Image received from left ZED camera - Size: %d%d", msg->width, msg->height);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "direction_node");
	ros::NodeHandle nh;

	ros::Subscriber utmSub = nh.subscribe<geometry_msgs::Pose2D>("/localization/pose_utm", 1 utmPubCallback);
	ros::Subscriber imgSub = nh.subscribe<>("zed/rgb/image_rect_color", 25, rgbImageCallback)
	ros::NodeHandle zed/rgb/image_rect_color;

	if (!ros::ok()) {
    		return -1;
  	}


	//need to determine rate at which we want to update co-ords

	ros::Rate loopRate(10);

	double latA, longA, latB, longB, X, Y, theta = 0;

	double latA = ; 
	double longA = ; 
	double latB = ; 
	double longB = ;

	double X = cos(latB) * sin(longB-longA);
	double Y = cos(latA) * sin(latB) - sin(latA) * cos(latB) * cos(longB-longA);

	//this is the bearing in degrees, 0 is north, 90 is east, 180 is south, 270 is west
	double theta = atan2(X, Y);



return 0;
}
