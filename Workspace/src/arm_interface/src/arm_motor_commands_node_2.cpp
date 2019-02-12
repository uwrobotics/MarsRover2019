#include <ros/ros.h>
#include <std_msgs/String.h> //includes string library 
#include <stdbool.h>
#include <list>
#include <vector>

struct msg_from_topics {
	bool ik_status; 
	std::list<float> data_points; 
};

class messageReceivedFromGUIWrapper {

	vector<msg_from_topics> msg;

	void messageReceivedFromGUI(const msg_from_topics values) {
		msg.add(values);
	}
	// To access certain index: msg.at(i); size: msg.size(); remove: msg.pop_back()
}

	



// messageRecievedFromGUI return the message that is recieved from the Inverse Kinematics library after it passes the message parameter into the library
// Not sure if this library exists or not yet
void messageRecievedFromGUI(const msg_from_topics values) {
		
	publisherFunctionToCAN(this, values, 10);
}

// messageReceieveFromCAN has a message passed into it and returrns the message 
void messageRecievedFromCAN(const std::list<float> values) {
	
	publisherFunctiontoGUI(this, values, 10); 
}

// subscriberFunction is passed into the necessary parameters required to initiaize the node and a boolean that determines if the message that is recieved from the topic
// needs to be passed into the Inverse Kinematics library or has it returned from it and needs to be returned as it is 
void subscriberFunctionFromGUI (ros::NodeHandle nh, int argc, char** argv) {
	//initializing the node
	ros::init(argc, argv, "subscribe_arm_motor_commands");

	ros::Subscriber sub = nh.subscribe("GUI", 10, messageReceivedFromGUI);

	ros::spinOnce();		
}

void subscriberFunctionFromCAN (ros::NodeHandle nh, int argc, char** argv) {
	//initializing the node
	ros::init(argc, argv, "subscribe_arm_motor_commands");

	ros::Subscriber sub = nh.subscribe("CAN", 10, messageReceivedFromCAN);

	ros::spinOnce();		
}


// publisher function consumes the parameters required to create the publisher object, the frequency at which publishing is carried out,
// and the topic which needs to be published to. 
void pubisherFunctionToCAN (ros::NodeHandle nh, msg_from_topics msg, int freq){
	//create publisher object
	ros::Publisher pub_user_to_can = nh.advertise<msg_from_topics>("CAN", 1000);
	ros::Rate rate(freq); 
	
	if (true) {
	//this should only run when ik_status is false, but this will be fixed when the inverseKinematicsLibrary is ready to be implemented i.e. "if (!msg.ik_status)"
	pub_user_to_can.publish(msg);
	}	
	else {
	//this should pass the msg into the inverse kinematics library and then publish the returned structure
	//pub_user_to_can.publish(inverseKinematicsLibrary(msg));
	}
}

void pubisherFunctionToGUI (ros::NodeHandle nh, std::list<float> msg, int freq){
	//create publisher object
	ros::Publisher pub_user_to_can = nh.advertise<std::list<float>>("GUI", 1000);
	ros::Rate rate(freq); 

	//publish message object and send message to rosout with details
	pub_user_to_can.publish(msg);
	
}

int main(int argc, char** argv) {
	//initializing the node
	ros::init(argc, argv, "arm_motor_commands");
	ros::NodeHandle nh;

	int freq = 2;
	

	//loop that publishes info until the node is shut down
	while (ros::ok()) {
		
		subscriberFunctionFromGUI(nh, argc, argv);

		subscriberFunctionFromCAN(nh, argc, argv);
		
		//wait until the next iteration call
		rate.sleep();
	}
}


