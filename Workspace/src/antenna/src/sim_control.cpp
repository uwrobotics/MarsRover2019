#include <ros/ros.h>
#include <std_msgs/Float32.h>

void angle_callback(const std_msgs::Float32::ConstPtr& msg)
{
	ROS_INFO("Receiving angle");
	ROS_INFO("angle from east: %f", msg->data);
}

int main (int argc, char *argv[])
{
    ROS_INFO("Starting sim control node");
    ros::init(argc, argv, "sim_control");
    ros::NodeHandle node_handle;
    
    // subscribe to rover angle from east
    ros::Subscriber angle_sub = node_handle.subscribe("/antenna/rover_angle_from_east", 1, angle_callback);

    ros::Rate loop_rate(10); // 10Hz update rate
    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce(); // Check for new messages
    }
}
