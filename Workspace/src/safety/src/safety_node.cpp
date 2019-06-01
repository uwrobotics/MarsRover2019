#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <iostream>
#include <fstream>

std::ofstream outFile("/home/nvidia/Desktop/safety_out.txt");


void current100Callback(can_msgs::FrameConstPtr frame) {
    uint8_t current = frame->data[0];
    std::stringstream strStream;
    strStream << "CURRENT_100A: " << current/5.0;
    outFile << strStream.str() << std::endl;
    ROS_DEBUG_STREAM_NAMED("SAFETY", strStream.str().c_str());
}
void current30Callback(can_msgs::FrameConstPtr frame) {
    uint8_t current = frame->data[0];
    std::stringstream strStream;
    strStream << "CURRENT_30A: " << current/10.0;
    outFile << strStream.str() << std::endl;
    ROS_DEBUG_STREAM_NAMED("SAFETY", strStream.str().c_str());
}
void batteryVoltageCallback(can_msgs::FrameConstPtr frame) {
    uint8_t voltage = frame->data[0];
    std::stringstream strStream;
    strStream << "BATTERY_VOLTAGE: " << voltage;
    outFile << strStream.str() << std::endl;
    ROS_DEBUG_STREAM_NAMED("SAFETY", strStream.str().c_str());
}
void eboxTempCallback(can_msgs::FrameConstPtr frame) {
    uint8_t temp = frame->data[0];
    std::stringstream strStream;
    strStream << "EBOX_TEMP: " << temp;
    outFile << strStream.str() << std::endl;
    ROS_DEBUG_STREAM_NAMED("SAFETY", strStream.str().c_str());
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "safety_node");
    ros::NodeHandle nh;
    
    

    ros::Subscriber current100Sub = nh.subscribe("/can/safety/current_sensor_100A", 10, current100Callback);
    ros::Subscriber current30Sub = nh.subscribe("/can/safety/current_sensor_30A", 10, current30Callback);
    ros::Subscriber batterySub = nh.subscribe("/can/safety/battery_voltage", 10, batteryVoltageCallback);
    ros::Subscriber eboxtempSub = nh.subscribe("/can/safety/ebox_temp", 10, eboxTempCallback);


    ros::spin();
    outFile.close();
    return 0;
}