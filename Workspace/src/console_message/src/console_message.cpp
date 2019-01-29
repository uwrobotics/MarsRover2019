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
	//Make a error folder in GitHub
	//ROS error macros and/or including error level in message
	//Compiling and running this (create a main for testing?)
	//Create a testing procedure
	//Next goal: link this library to other nodes
	//Other goals: see something in Qt console

};

Console_Message::Console_Message(void){
	//initilaize/create the publisher pub
	ros::Publisher pub;

	//create topic
	pub = node_handle.advertise<std_msgs::String>("Messages", 1);
}

void Console_Message::sendMessage(string message, string level){
	
	//initializew message instance
	std_msgs::String msg;

	//convert message into something?
	msg.data = <std_msgs::String>(message);

	//convert level into ROS macro? ask Melvin?
	if(level != "NULL"){
		//set a macro? combine the message and level?
	}
	//use publisher to send error message to topic
	pub.publish(msg);
	

}
