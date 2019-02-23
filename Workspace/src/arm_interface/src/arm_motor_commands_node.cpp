#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <std_msgs/Int32.h>
#include <stdbool.h>
#include <std_msgs/Float64MultiArray.h>
#include <arm_node/Custom_msg.h>

class InitObjects {
	public:
	static ros::NodeHandle nh;  
        static ros::Publisher pub_user_to_GUI;
	static ros::Publisher pub_user_to_CAN;   
        static int freq;
	
	
	/**
	InitObjects(): 
		pub_user_to_GUI =  nh.advertise<std_msgs::Float64MultiArray>("GUI", 1000);
       		pub_user_to_CAN =  nh.advertise<arm_node::Custom_msg>("CAN", 1000);
		freq = 10;
	{
	}
	**/

	
	static ros::Publisher getPubGUI() {
		return pub_user_to_GUI; 
	}
	static ros::Publisher getPubCAN() {
		return pub_user_to_CAN; 
	}
	static ros::NodeHandle getNodeHandlerObject() {
		return nh; 
	}	
	static int getFreq() {
		return freq; 
	}	
};


// publisher function consumes the parameters required to create the publisher object, the frequency at which publishing is carried out,
// and the topic which needs to be published to. 
void publisherFunctionToCAN (arm_node::Custom_msg msg){
	//create publisher object
	ros::Rate rate(InitObjects::getFreq()); 
	
	if (msg.ik_status == false) {
	//this should only run when ik_status is false, but this will be fixed when the inverseKinematicsLibrary is ready to be implemented i.e. "if (!msg.ik_status)"
	InitObjects::getPubCAN().publish(msg);
	}	
	else {
	//this should pass the msg into the inverse kinematics library and then publish the returned structure
	//InitObjects.getPubCAN().publish(inverseKinematicsLibrary(msg));
	}
}

void publisherFunctionToGUI (std_msgs::Float64MultiArray msg){
	//create publisher object
	ros::Rate rate(InitObjects::getFreq()); 

	//publish message object and send message to rosout with details
	InitObjects::getPubGUI().publish(msg);
	
}

// messageRecievedFromGUI return the message that is recieved from the Inverse Kinematics library after it passes the message parameter into the library
// Not sure if this library exists or not yet
void messageReceivedFromGUI(const arm_node::Custom_msg values) {
	
	publisherFunctionToCAN(values);
}

// messageReceieveFromCAN has a message passed into it and returrns the message 
void messageReceivedFromCAN(const std_msgs::Float64MultiArray values) {
	
	publisherFunctionToGUI(values); 
}

// subscriberFunction is passed into the necessary parameters required to initiaize the node and a boolean that determines if the message that is recieved from the topic

// needs to be passed into the Inverse Kinematics library or has it returned from it and needs to be returned as it is 
void subscriberFunctionFromGUI (int argc, char** argv) {
	
	ros::Subscriber sub = InitObjects::getNodeHandlerObject().subscribe("GUI", 10, &messageReceivedFromGUI);

	ros::spinOnce();		
}

void subscriberFunctionFromCAN (int argc, char** argv) {
	//initializing the node
	ros::init(argc, argv, "subscribe_arm_motor_commands");


	ros::Subscriber sub = InitObjects::getNodeHandlerObject().subscribe("CAN", 10, &messageReceivedFromCAN);

	ros::spinOnce();		
}

int main(int argc, char** argv) {
	//initializing the node
	ros::init(argc, argv, "arm_motor_commands");		

	InitObjects::getPubGUI() =  InitObjects::getNodeHandlerObject().advertise<std_msgs::Float64MultiArray>("GUI", 1000);
	InitObjects::getPubCAN() =  InitObjects::getNodeHandlerObject().advertise<arm_node::Custom_msg>("CAN", 1000);
	InitObjects::getFreq() = 10;

	int freq = 2;
	ros::Rate rate(freq);

	//loop that publishes info until the node is shut down
	while (ros::ok()) {
		
		subscriberFunctionFromGUI(argc, argv);

		subscriberFunctionFromCAN(argc, argv);
		
		//wait until the next iteration call
		rate.sleep();
	}
}
