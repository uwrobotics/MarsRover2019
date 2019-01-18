#include <ros/ros.h>
#include <arm_interface/commands.h>	//this will allow us to create an object that stores the information taken from the GUI robot commands
#include <inverse_kinematics/message.h>		//this will be the object that stores all 6 motor speeds and 6 angles that are spit out of the inverse kinematics library
#include <string> //includes string library 


int main(int argc, char** argv) {
	//initializing the node
	ros::init(argc, argv, "arm_motor_commands");
	ros::NodeHandle nh;

	int freq = 2;
	

	//loop that publishes info until the node is shut down
	while (ros::ok()) {
		
		inverse_kinematics:message msgFromGUI = subscriberFunction(nh, argc, argv, "GUI", true); 
		publisherFunction(nh, msgFromGUI, freq, "CAN");


		inverse_kinematics:message msgCalculatedAfterSubscription = subscriberFunction(nh, argc, argv, "CAN", false); 
		publisherFunction(nh, msgCalculated, freq, "GUI"); 


		//wait until the next iteration call
		rate.sleep();
	}
}

inverse_kinematics::message messageRecievedFromGUI(const arm_interface::commands msg) {
	return inverseK(msg);
}

inverse_kinematics::message messageRecievedFromIK(const arm_interface::commands msg) {
	return msg; 
}

inverse_kinematics::message subscriberFunction (ros::NodeHandle nh, int argc, char** argv, std::string topic, std::bool SendToIK) {
	//initializing the node
	ros::init(argc, argv, "subscribe_arm_motor_commands");

	if (SendToIK) {
		//create a subscriber object
		ros::Subscriber sub = nh.subscribe(topic, 10, &messageFromGUI);	
	}
	
	else {
		//create a subscriber object
		ros::Subscriber sub = nh.subscribe(topic, 10, &messageRecievedFromIK);		
	}

	return ros::spin();		
}

inverse_kinematics::message inverseK(arm_interface::commands msg) {
	return inverse_kinematics::message.ik(msg);	//where ik is the method in the library that returns optimal motor speeds and angles
}

void pubisherFunction (ros::NodeHandle nh, inverse_kinematics::message msg, int freq, std::string topic){
	//create publisher object
	ros::Publisher pub_user_to_can = nh.advertise<inverse_kinematics::message>(topic, 1000);
	ros::Rate rate(freq);

	//publish message object and send message to rosout with details
	pub_user_to_can.publish(msg);
	
}
