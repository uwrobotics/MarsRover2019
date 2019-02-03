#include <ros/ros.h>
#include <std_msgs/String.h> //includes string library 
#include <std_msgs/Float32MultiArray>
#include <std_msgs/Bool.h>

struct msg_from_topics {
	std::Bool ik_status; 
	std::Float32MultiArray data_points[7]; 
};


int main(int argc, char** argv) {
	//initializing the node
	ros::init(argc, argv, "arm_motor_commands");
	ros::NodeHandle nh;

	int freq = 2;
	

	//loop that publishes info until the node is shut down
	while (ros::ok()) {
		
		msg_from_topics current_msg_from_GUI;
		std::Float32MultiArray data = subscriberFunction(nh, argc, argv, "GUI");

		for (int i = 0; i < 4; i++) {
			current_msg_from_GUI.data_points[i] = data[i];
		}
 
		current_msg_from_GUI.ik_status = false; 
		publisherFunction(nh, current_msg_from_GUI, freq, "CAN");

		msg_from_topics current_msg_from_IK;
		std::Float32MultiArray data_2 = subscriberFunction(nh, argc, argv, "CAN");
		
		for (int i = 0; i < 7; i++) {
			current_msg_from_IK.data_points[i] = data_2[i];
		}

		current_msg_from_IK.ik_status = true;
		publisherFunction(nh, current_msg_from_IK, freq, "CAN");
		publisherFunction(nh, current_msg_from_IK, freq, "GUI"); 

		//wait until the next iteration call
		rate.sleep();
	}
}

// messageRecievedFromGUI return the message that is recieved from the Inverse Kinematics library after it passes the message parameter into the library
// Not sure if this library exists or not yet
std_msgs::Float32MultiArray messageRecievedFromGUI(const std_msgs:Float32MultiArray values) {
	return values; 
}

// messageReceieveFromIK has a message passed into it and returrns the message 
std_msgs::Float32MultiArray messageRecievedFromIK(const std_msgs:Float32MultiArray values) {
	return values; 
}

// subscriberFunction is passed into the necessary parameters required to initiaize the node and a boolean that determines if the message that is recieved from the topic
// needs to be passed into the Inverse Kinematics library or has it returned from it and needs to be returned as it is 
std_msgs::Float32MultiArray subscriberFunction (ros::NodeHandle nh, int argc, char** argv, std_msgs::String topic) {
	//initializing the node
	ros::init(argc, argv, "subscribe_arm_motor_commands");

	if (topic.compare("GUI") == 0) {
		//create a subscriber object
		ros::Subscriber sub = nh.subscribe(topic, 10, messageReceivedFromGUI);	
	}
	
	else {
		//create a subscriber object
		ros::Subscriber sub = nh.subscribe(topic, 10, messageRecievedFromIK);		
	}

	return ros::spin();		
}


// publisher function consumes the parameters required to create the publisher object, the frequency at which publishing is carried out,
// and the topic which needs to be published to. 
void pubisherFunction (ros::NodeHandle nh, std_msgs::Float32MultiArray msg_joint_angles, int freq, std_msgs::String topic){
	//create publisher object
	ros::Publisher pub_user_to_can = nh.advertise<std_msgs::Float32MultiArray>(topic, 1000);
	ros::Rate rate(freq); 

	//publish message object and send message to rosout with details
	pub_user_to_can.publish(msg_joint_angles);
	
}
