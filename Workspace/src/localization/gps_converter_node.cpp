#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

ros::Publisher pub;

void gpsCallback(sensor_msgs::NavSatFixPtr gps) {
  gps->header.frame_id = "gps";
  pub.publish(gps);
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "gps_converter_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/gps/fix", 1, gpsCallback);
  pub = nh.advertise<sensor_msgs::NavSatFix>("/rover/fix", 1);

  ros::spin();

  return 0;
}
