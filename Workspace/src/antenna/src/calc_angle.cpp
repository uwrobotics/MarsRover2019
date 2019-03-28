#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <cmath>
#include <std_msgs/Float32.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose2D.h>
#include <tf2_ros/buffer.h>
#include <robot_localization/navsat_conversions.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#define MAX_ANTENNA_FIXES   10

double antenna_x = 0;                           // Positive to the right (eastward)
double antenna_x_sum = 0;
double antenna_y = 0;                           // Positive facing up (northward)
double antenna_y_sum = 0;
double rover_x = 0;                             // Positive to the right (eastward)
double rover_y = 0;                             // Positive facing up (northward)
uint8_t num_of_antenna_pings = 0;

ros::Publisher angle_publisher;

//void antenna_loc_callback(const sensor_msgs::NavSatFix::ConstPtr& ptr)
//{
//    // Average the first few fixes to get a more accurate permanent location of the antenna
//    if (num_of_antenna_pings < MAX_ANTENNA_FIXES)
//    {
//        ROS_INFO("Received antenna location topic");
//        std::string UTMZone;
//        double utm_east, utm_north;
//        RobotLocalization::NavsatConversions::LLtoUTM(ptr->latitude, ptr->longitude,
//                utm_north, utm_east, UTMZone); // Converts lat/long to UTM east/west

//        antenna_x_sum += utm_east;
//        antenna_y_sum += utm_north;

//        antenna_x = antenna_x_sum / ++num_of_antenna_pings;
//        antenna_y = antenna_y_sum / num_of_antenna_pings;
//    }
//}

// utm version
void rover_loc_utm_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    // waits for antenna location, not sure how to get this....
    // for now can just use an arbitrary fixed location as the antenna location for testing
    antenna_x = 21;
    antenna_y = 30;
//    if (num_of_antenna_pings == 0)
//    {
//        ROS_DEBUG("No antenna ping received yet");
//        // Haven't received antenna location yet, do nothing
//        return;
//    }
//    else
//    {
        // Find dx and dy based on antenna locations and rover locations, and use arctan to find angle.
        // Assume antenna is pointing north?
        ROS_INFO("Received rover location topic");
//        std::string UTMZone;
//        double utm_east, utm_north;
//        RobotLocalization::NavsatConversions::LLtoUTM(ptr->latitude, ptr->longitude,
//                rover_y, rover_x, UTMZone); // Converts lat/long to UTM east/west

        // set rover x and y positions from message
        rover_x = msg->x;
        rover_y = msg->y;

        // Get vector of robot's position relative to Antenna
        double diff_x = rover_x - antenna_x;
        double diff_y = rover_y - antenna_y;
        ROS_INFO ("Rover_x: %f,     Rover_y: %f", rover_x, rover_y);
        ROS_INFO ("Antenna_x: %f,     Antenna_y: %f", antenna_x, antenna_y);
        ROS_INFO ("Diff_x: %f,     Diff_y: %f", diff_x, diff_y);

        //Get rover's angle from the east-west axis
        double rover_angle_from_east = atan(diff_y/diff_x) * 180 / M_PI; // Convert to degrees
        ROS_INFO ("rover_angle_from_east: %f", rover_angle_from_east);

        // publish angle for pi to use
        std_msgs::Float32 angle_msg;
        angle_msg.data = rover_angle_from_east;
        angle_publisher.publish(angle_msg);
   // }
}

// using ekf filtered data
void rover_loc_filtered_callback(const nav_msgs::Odometry::ConstPtr& msg) {
//     ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
//     ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    
    rover_x = msg->pose.pose.position.x;
    rover_y = msg->pose.pose.position.y;
    
    // random fixed antenna location for now
    antenna_x = -6.5;
    antenna_y = -6.5;
    
    // convert quaternion orientation to just 2D yaw (don't really need this if using relative angles)
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ROS_INFO("Yaw: [%f]", yaw);
    
    // Get vector of robot's position relative to Antenna
    double diff_x = rover_x - antenna_x;
    double diff_y = rover_y - antenna_y;
    ROS_INFO ("Rover_x: %f,     Rover_y: %f", rover_x, rover_y);
    ROS_INFO ("Antenna_x: %f,     Antenna_y: %f", antenna_x, antenna_y);
    ROS_INFO ("Diff_x: %f,     Diff_y: %f", diff_x, diff_y);
    
    //Get rover's angle from the east-west axis
    double rover_angle_from_east = atan(diff_y/diff_x) * 180 / M_PI; // Convert to degrees
    ROS_INFO ("rover_angle_from_east: %f", rover_angle_from_east);

    // publish angle for pi to use
    std_msgs::Float32 angle_msg;
    angle_msg.data = rover_angle_from_east;
    angle_publisher.publish(angle_msg);
}

int main (int argc, char *argv[])
{
    ROS_INFO("Starting angle node");
    ros::init(argc, argv, "calc_angle");
    ros::NodeHandle node_handle;

    // subscribe to the utm coordinates being published from the rover
    //ros::Subscriber rover_loc_sub = node_handle.subscribe("/localization/pose_utm", 1, rover_loc_callback);
    
    // subscribe to filtered odometry info 
    ros::Subscriber rover_loc_sub = node_handle.subscribe("/odometry/filtered", 1, rover_loc_filtered_callback);

    // publish angle
    angle_publisher = node_handle.advertise<std_msgs::Float32>("/antenna/rover_angle_from_east", 10);
//    ros::Subscriber antenna_sub = node_handle.subscribe("antenna/fix", 1, antenna_loc_callback);
//    ros::Subscriber rover_sub = node_handle.subscribe("/navsat/fix", 1, rover_loc_callback);

    ros::Rate loop_rate(10); // 20Hz update rate
    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce(); // Check for new messages
    }
}
