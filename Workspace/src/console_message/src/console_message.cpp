#include <console_message/console_message.h>


//Questions and future plans:
//GET THE CODE TO COMPILE
//Finish test_console_message/make it useful to us
//Next goal: link this library to other nodes - start with linking a test program/project/node/thing
//Other goals: see something in Qt console
ros::Publisher* ConsoleMessage::s_pPub = nullptr;

//Console_Message::Console_Message(void){
//	//initilaize/create the publisher pub
//	ros::NodeHandle node_handle;
//	ros::Publisher info_pub, warn_pub, error_pub, fatal_pub, debug_pub;
//
//	//create topics
//	info_pub = node_handle.advertise<std_msgs::String>("Info", 100);
//	warn_pub = node_handle.advertise<std_msgs::String>("Warning", 100);
//	error_pub = node_handle.advertise<std_msgs::String>("Error", 100);
//	fatal_pub = node_handle.advertise<std_msgs::String>("Fatal", 100);
//	debug_pub = node_handle.advertise<std_msgs::String>("Debug", 100);
//}

void ConsoleMessage::Initialize(ros::NodeHandle &nh) {
	s_pPub = new ros::Publisher(nh.advertise<console_message::console_msg>("/console_msgs", 1));
	ROS_INFO("console initialized");
}

void ConsoleMessage::SendMessage(std::string msg_text, eLevels level) {
	console_message::console_msg msg;
	msg.message = msg_text;
	msg.level = level;
	msg.sender = ros::this_node::getName();
	msg.time = ros::Time::now().toSec();

	s_pPub->publish(msg);

}
