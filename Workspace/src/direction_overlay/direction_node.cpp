#include <iostream>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <sensor_msgs/NavSatFix.h>


using namespace cv;
using namespace std;

void NavSatCallback(sensor_msgs::NavSatFixConstPtr msg) { pNavSatOrigin = msg; }

int main(int argc, char **argv) {
	ros::init(argc, argv, "direction_node");
	ros::NodeHandle n;

	ros::Subscriber sub = zed/rgb/image_rect_color.subscribe("ZEDimg", 25)
	ros::Subscriber sub = zed/rgb/image_rect_color.subscribe("ZEDimg", 25)
	ros::Subscriber sub = zed/rgb/image_rect_color.subscribe("ZEDimg", 25)
	ros::NodeHandle zed/rgb/image_rect_color;

	//need data structure to store incoming co-ords

	int latA, longA, latB, longB, X, Y, theta = 0;

	int latA = ; //average of latitude of point A
	int longA = ; //average of longitude of point A
	int latB = ; //average of latitude of point B
	int longB = ; //average of longitude of point B

	int X = cos(latB) * sin(longB-longA);
	int Y = cos(latA) * sin(latB) - sin(latA) * cos(latB) * cos(longB-longA);

	//this is the bearing in degrees, 0 is north, 90 is east, 180 is south, 270 is west
	int theta = atan2(X, Y);



return 0;
}
