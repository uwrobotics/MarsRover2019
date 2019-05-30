#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <cmath>

#define MOVE_THRESH 1
#define CW 0
#define CCW 1

float antenna_position = 0;
float target_angle = 0;
float target_angle_rad = 0;

int turn_dir;
float speed;

float simulate_motor(int turn_dir, int speed);
ros::Publisher publisher;

void angle_callback(const std_msgs::Float32::ConstPtr& msg)
{
	ROS_INFO("Target angle: %f", msg->data);

	if (msg->data >= 0)
		target_angle = msg->data;
	else
		target_angle = 360 + msg->data;

	ROS_INFO("Target angle filtered: %f", target_angle);

	target_angle_rad = target_angle * M_PI / 180.0;

	if (fabs(target_angle - antenna_position) > MOVE_THRESH) // don't bother moving if angle difference is very low
	{
		// figure out which direction is quicker to turn in to reach target angle
		if (antenna_position < target_angle) {
			if ((target_angle - antenna_position) > 180)
				turn_dir = CW;
			else
				turn_dir = CCW;
		}
		else {
			if ((antenna_position - target_angle) > 180) 
				turn_dir = CCW;
			else
				turn_dir = CW;
		}
	}

	// publish target position to simulation
    std_msgs::Float64 pos_msg;
    pos_msg.data = simulate_motor(turn_dir, 0) + target_angle_rad;
    //pos_msg.data = target_angle * M_PI / 180.0;
    publisher.publish(pos_msg);
    ROS_INFO("Sending dest position: %f", pos_msg.data * 180.0 / M_PI);
}

void antenna_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
	//ROS_INFO("Receiving antenna angle, current antenna angle, unfiltered: %f", msg->position[0] * 180.0 / M_PI);
	// antenna position is given as negative past 180 in gazebo, ie 181 is -179 so adjust for this
	if (msg->position[0] >= 0)
		antenna_position = msg->position[0] * 180.0 / M_PI;
	else
		antenna_position = 360 + (msg->position[0] * 180.0 / M_PI);

	ROS_INFO("Receiving antenna angle, current antenna angle: %f", antenna_position);
}

float simulate_motor(int dir, int speed) {
	// simulate motor forward/ motor back by incrementing/decrementing position slightly
	float pos_change;

	if (dir == CW)
		pos_change = -0.001; // motor off
	else
		pos_change = 0.001; // motor on
	return pos_change;

}

int main (int argc, char *argv[])
{
    ROS_INFO("Starting sim control node");
    ros::init(argc, argv, "sim_control");
    ros::NodeHandle node_handle;
    
    // subscribe to rover angle from east
    ros::Subscriber angle_sub = node_handle.subscribe("/antenna/rover_angle_from_east", 1, angle_callback);

    // subscribe to current antenna position
    ros::Subscriber antenna_angle_sub = node_handle.subscribe("/antenna_robot/joint_states", 1, antenna_callback);

    // publish to simulated antenna
   publisher = node_handle.advertise<std_msgs::Float64>("/antenna_robot/antenna_sim_antenna_controller/command", 10);

    ros::Rate loop_rate(10); // 10Hz update rate
    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce(); // Check for new messages
    }
}
