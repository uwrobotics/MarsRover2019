#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <opencv2/core.hpp>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
//Inspired from: https://stackoverflow.com/a/16083336/8245487
//And : https://github.com/stereolabs/zed-ros-wrapper/blob/master/tutorials/zed_depth_sub_tutorial/src/zed_depth_sub_tutorial.cpp
class DisparityMapReader{
	public:
	DisparityMapReader(ros::NodeHandle* node):disparityMap(NULL), mapWidth(0), mapHeight(0), n(node){
		subDisparityMap = n->subscribe("/zed/disparity/disparity_image", 10, disparityCallback);
	}

	void disparityCallback(const sensor_msgs::Image::ConstPtr& msg)
	{
		disparityMap = (float*)(&msg->data[0]);
		mapWidth = msg->width;
		mapHeight = msg->height;
	}

	float* getDisparityMap(){
		return disparityMap;
	}		

	int getMapWidth(){
		return mapWidth;
	}

	int getMapHeight(){
		return mapHeight;
	}

	/*
	void run(){
		
	}
*/
		
	private:
	double* disparityMap;
	int mapWidth;
	int mapHeight;
	ros::NodeHandle* n;
	ros::Subscriber subDisparityMap;
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "obstacle_detection");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */

	/**
	 * The subscribe() call is how you tell ROS that you want to receive messages
	 * on a given topic.  This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing.  Messages are passed to a callback function, here
	 *t called chatterCallback.  subscribe() returns a Subscriber object that you
	 * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
	 * object go out of scope, this callback will automatically be unsubscribed from
	 * this topic.
	 *
	 * The second parameter to the subscribe() function is the size of the message
	 * queue.  If messages are arriving faster than they are being processed, this
	 * is the number of messages that will be buffered up before beginning to throw
	 * away the oldest ones.
	 */
	// %Tag(SUBSCRIBER)%
	// %EndTag(SUBSCRIBER)%
	ros::NodeHandler n;	
	ros::Rate r(10);
	DisparityMapReader d(&n);
	while(1){
		float* map = d.getDisparityMap();	
		cv::Mat mat(d.getMapHeight(),d.getMapWidth(), CV_8UC1, map); 	
		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}
// %EndTag(FULLTEXT)%
