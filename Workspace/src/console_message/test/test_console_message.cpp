#include <console_message.h> //change this
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv[]) {
	ros::init(argc, argv, "error_node");
	ros::NodeHandle node_handle;

	//Linking the node to the library?

	Console_Message msg;
	struct Message argument;
	argument.node_sender = "Test_Node";
	
	//Test 1 - Info Topic
	argument.msg = "Testing!";
	argument.level = "Info";
	msg.sendMessage(argument);

	//Test 2 - Warning Topic
	argument.msg = "Test2";
	argument.level = "Warning";
	msg.sendMessage(argument);

	//Test 3 - Error Topic
	argument.msg = "Test 3";
	argument.level = "Error";
	msg.sendMessage(argument);

	//Test 4 - Fatal Topic
	argument.msg = "test 4";
	argument.level = "Fatal";
	msg.sendMessage(argument);

	//Test 5 - Debug Topic
	argument.msg = "test 5";
	argument.level = "Debug";
	msg.sendMessage(argument);
	

}
