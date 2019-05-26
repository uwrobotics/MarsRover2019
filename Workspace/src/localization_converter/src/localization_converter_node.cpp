#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <console_message/console_message.h>
#include <console_message/console_msg.h>
#include <robot_localization/SetDatum.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

tf2_ros::Buffer tfBuffer;

sensor_msgs::NavSatFixConstPtr pNavSatOrigin = nullptr;

void NavSatCallback(sensor_msgs::NavSatFixConstPtr msg) { pNavSatOrigin = msg; }

// Helper to fill in pose messages
void CreatePoseMsgForFrame(const std::string &worldFrameId,
                           geometry_msgs::Pose2D &poseMsg) {
  geometry_msgs::TransformStamped roverLocToFrame;
  try {
    roverLocToFrame = tfBuffer.lookupTransform(worldFrameId, "base_link",
                                               ros::Time(0), ros::Duration(2));
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
  }

  double curGpsUtmX = roverLocToFrame.transform.translation.x;
  double curGpsUtmY = roverLocToFrame.transform.translation.y;

  double heading = 0;

  heading = tf::getYaw(roverLocToFrame.transform.rotation); // - M_PI / 2;
  if (heading < -M_PI) {
    heading += 2 * M_PI;
  }

  // ROS_INFO("X: %f, Y: %f, Heading: %f",curGpsUtmX, curGpsUtmY, heading);

  poseMsg.x = curGpsUtmX;
  poseMsg.y = curGpsUtmY;
  poseMsg.theta = heading;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "localization_converter");
  ros::NodeHandle nh;

  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Subscriber navSatSub = nh.subscribe("/navsat/fix", 1, NavSatCallback);

  ros::Publisher utmPub =
      nh.advertise<geometry_msgs::Pose2D>("/localization/pose_utm", 1);
  ros::Publisher mapPub =
      nh.advertise<geometry_msgs::Pose2D>("/localization/pose_map", 1);
  ros::Publisher odomPub =
      nh.advertise<geometry_msgs::Pose2D>("/localization/pose_odom", 1);

  ConsoleMessage::Initialize(nh);

  ros::Rate loopRate(10);

  // Wait until we get coord so we can manually set datum
  while (ros::ok() && !pNavSatOrigin) {
    loopRate.sleep();
    ros::spinOnce();
  }
  if (!ros::ok()) {
    return -1;
  }

  // Set the datum
  ros::ServiceClient datumClient =
      nh.serviceClient<robot_localization::SetDatum>("/datum");
  if (!datumClient.exists()) {
    ROS_WARN("Datum service does not exist. Waiting...");
    datumClient.waitForExistence();
  }
  robot_localization::SetDatum request;
  request.request.geo_pose.position.latitude = pNavSatOrigin->latitude;
  request.request.geo_pose.position.longitude = pNavSatOrigin->longitude;
  request.request.geo_pose.orientation.w = 1.0;

  ROS_INFO("Setting Datum");
  ConsoleMessage::SendMessage("Localization setting datum");
  if (!datumClient.call(request)) {
    ROS_ERROR("SetDatum failed");
    ConsoleMessage::SendMessage("Localization datum set failed",
                                ConsoleMessage::ERROR);
    return -1;
  }

  // Publish pose in map and utm frames
  while (ros::ok()) {
    geometry_msgs::Pose2D utmPose;
    CreatePoseMsgForFrame("utm", utmPose);
    utmPub.publish(utmPose);

    geometry_msgs::Pose2D mapPose;
    CreatePoseMsgForFrame("map", mapPose);
    mapPub.publish(mapPose);

    geometry_msgs::Pose2D odomPose;
    CreatePoseMsgForFrame("odom", odomPose);
    odomPub.publish(odomPose);

    loopRate.sleep();
  }

  return 0;
}
