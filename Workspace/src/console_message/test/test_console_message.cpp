#include <console_message.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv[]) {
	ros::init(argc, argv, "error_node");
	ros::NodeHandle node_handle;

	//Linking the node to the library?
	
	//Test 1
	Console_Message msg;
	message_t argument;
	argument.node_sender = "Test_Node";
	argument.msg = "Testing!";
	argument.level = "Info";
	msg.sendMessage(argument);

	//Test 2
	argument.node_sender = "Test_Node";
	argument.msg = "Test2";
	argument.level = "Warning";
	msg.sendMessage(argument);

}
