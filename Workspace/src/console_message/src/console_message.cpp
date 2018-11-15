#include <console_message.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>

class Console_Message
{

	public:

	Console_Message();

	void sendMessage(string message, string level);

	//Questions and future plans:
	//Compiling and running this (create a main for testing?)
	//Create a testing procedure
	//Next goal: link this library to other nodes
	//Other goals: see something in Qt console

};

Console_Message::Console_Message(void){
	//initilaize/create the publisher pub
	ros::Publisher info_pub, warn_pub, error_pub, fatal_pub, debug_pub;

	//create topics
	info_pub = node_handle.advertise<std_msgs::String>("Info", 1);
	warn_pub = node_handle.advertise<std_msgs::String>("Warning", 1);
	error_pub = node_handle.advertise<std_msgs::String>("Error", 1);
	fatal_pub = node_handle.advertise<std_msgs::String>("Fatal", 1);
	debug_pub = node_handle.advertise<std_msgs::String>("Debug", 1);
}

void Console_Message::sendMessage(message_t arg){

	//initialize message instance
	std_msgs::String msg;

	switch(arg.level){

	  case "Info" :
		string sender_message = arg.node_sender + arg.msg;
	  	msg.data = <std_msgs::String>(sender_message);
		//send to Info topic
		info_pub.publish(msg);
		break;

	  case "Warning" :
		string sender_message = arg.node_sender + arg.msg;
	  	msg.data = <std_msgs::String>(sender_message);
		//send to Warning topic
		warn_pub.publish(msg);
		break;

	  case "Error" :
		string sender_message = arg.node_sender + arg.msg;
	  	msg.data = <std_msgs::String>(sender_message);
		//send to Error topic
		error_pub.publish(msg);
		break;

	  case "Fatal" :
		string sender_message = arg.node_sender + arg.msg;
	  	msg.data = <std_msgs::String>(sender_message);
		//send to Fatal topic
		fatal_pub.publish(msg);
		break;

	  case "Debug" :
		string sender_message = arg.node_sender + arg.msg;
	  	msg.data = <std_msgs::String>(sender_message);
		//send to Debug topic
		debug_pub.publish(msg);
		break;

	//need a default case? return that the given level doesn't match what we have?

	}

}
