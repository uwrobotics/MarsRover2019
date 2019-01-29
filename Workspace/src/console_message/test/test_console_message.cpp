#include <console_message.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv[]) {
	ros::init(argc, argv, "error_node");
	ros::NodeHandle node_handle;

	//Linking the node to the library?
	
	//Test 1
	Console_Message msg;
	msg.sendMessage("Testing!", "NULL");

	//Test 2
	msg.sendMessage("Testing levels", "Warning");

}
