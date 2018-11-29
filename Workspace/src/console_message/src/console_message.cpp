#include "console_message.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cstring>


//Questions and future plans:
//GET THE CODE TO COMPILE
//Finish test_console_message/make it useful to us
//Next goal: link this library to other nodes - start with linking a test program/project/node/thing
//Other goals: see something in Qt console


Console_Message::Console_Message(void){
	//initilaize/create the publisher pub
	ros::NodeHandle node_handle;
	ros::Publisher info_pub, warn_pub, error_pub, fatal_pub, debug_pub;

	//create topics
	info_pub = node_handle.advertise<std_msgs::String>("Info", 100);
	warn_pub = node_handle.advertise<std_msgs::String>("Warning", 100);
	error_pub = node_handle.advertise<std_msgs::String>("Error", 100);
	fatal_pub = node_handle.advertise<std_msgs::String>("Fatal", 100);
	debug_pub = node_handle.advertise<std_msgs::String>("Debug", 100);
}

void Console_Message::sendMessage(struct Message arg){

	//initialize message instance
	std_msgs::String msg;
	char* sender = strcat(arg.node_sender, " - ");
	char* sender_message = strcat(sender, arg.msg);
	msg.data = <std_msgs::String>(sender_message);

	int topic = 0;
	if(strcmp(arg.level, "Info") == 0){
	  topic = 1;
	}
	else if(strcmp(arg.level, "Warning") == 0){
	  topic = 2;
	}
	else if(strcmp(arg.level, "Error") == 0){
	  topic = 3;
	}
	else if(strcmp(arg.level, "Fatal") == 0){
	  topic = 4;
	}
	else if(strcmp(arg.level, "Debug") == 0){
	  topic = 5;
	}
	else{
	  //warning? set another number?
	}




	switch(topic){

	  case 1 :
		
		//send to Info topic
		info_pub.publish(msg);
		break;

	  case 2 :
		//send to Warning topic
		warn_pub.publish(msg);
		break;

	  case 3 :
		//send to Error topic
		error_pub.publish(msg);
		break;

	  case 4 :
		//send to Fatal topic
		fatal_pub.publish(msg);
		break;

	  case 5 :
		//send to Debug topic
		debug_pub.publish(msg);
		break;

	//need a default case? return that the given level doesn't match what we have?

	}

}
