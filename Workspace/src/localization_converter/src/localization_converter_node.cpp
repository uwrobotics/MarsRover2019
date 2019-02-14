#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <robot_localization/SetDatum.h>


tf2_ros::Buffer tfBuffer;


sensor_msgs::NavSatFixConstPtr pNavSatOrigin = nullptr;

void NavSatCallback(sensor_msgs::NavSatFixConstPtr msg)
{
  pNavSatOrigin = msg;
}

// Helper to fill in pose messages
void CreatePoseMsgForFrame(const std::string& worldFrameId, geometry_msgs::Pose2D& poseMsg)
{
  geometry_msgs::TransformStamped roverLocToFrame;
  try {
    roverLocToFrame = tfBuffer.lookupTransform(
        worldFrameId,
        "base_link", ros::Time(0),
        ros::Duration(2));
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
  }


  double curGpsUtmX = roverLocToFrame.transform.translation.x;
  double curGpsUtmY = roverLocToFrame.transform.translation.y;

  double heading = 0;

  heading = tf::getYaw(roverLocToFrame.transform.rotation);// - M_PI / 2;
  if (heading < -M_PI) {
    heading += 2 * M_PI;
  }

  //ROS_INFO("X: %f, Y: %f, Heading: %f",curGpsUtmX, curGpsUtmY, heading);

  poseMsg.x = curGpsUtmX;
  poseMsg.y = curGpsUtmY;
  poseMsg.theta = heading;
}

// create sample messages to simulate rover movement for testing antenna
// Helper to fill in pose messages
void CreateSamplePoseMsg(geometry_msgs::Pose2D& poseMsg, int x, int y)
{
  //ROS_INFO("X: %f, Y: %f, Heading: %f",curGpsUtmX, curGpsUtmY, heading);
  poseMsg.x = x;
  poseMsg.y = y;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "localization_converter");
  ros::NodeHandle nh;

  //tf2_ros::TransformListener tfListener(tfBuffer);

  //ros::Subscriber navSatSub = nh.subscribe("/navsat/fix", 1, NavSatCallback);

  ros::Publisher utmPub = nh.advertise<geometry_msgs::Pose2D>("/localization/pose_utm", 1);
  //ros::Publisher mapPub = nh.advertise<geometry_msgs::Pose2D>("/localization/pose_map",1);

  ros::Rate loopRate(10);

//  // Wait until we get coord so we can manually set datum
//  while (ros::ok() && !pNavSatOrigin)
//  {
//    ROS_INFO("Waiting for coordinates to set datum");
//    loopRate.sleep();
//    ros::spinOnce();
//  }
//  if (!ros::ok())
//  {
//    return -1;
//  }

//  // Set the datum
//  ros::ServiceClient datumClient = nh.serviceClient<robot_localization::SetDatum>("/datum");
//  if (!datumClient.exists())
//  {
//    ROS_WARN("Datum service does not exist. Waiting...");
//    datumClient.waitForExistence();
//  }
//  robot_localization::SetDatum request;
//  request.request.geo_pose.position.latitude = pNavSatOrigin->latitude;
//  request.request.geo_pose.position.longitude = pNavSatOrigin->longitude;
//  request.request.geo_pose.orientation.w = 1.0;

//  ROS_INFO("Setting Datum");
//  if(!datumClient.call(request))
//  {
//    ROS_ERROR("SetDatum failed");
//    return -1;
//  }


//  // Publish pose in map and utm frames
//  while(ros::ok()) {
//    geometry_msgs::Pose2D utmPose;
//    CreatePoseMsgForFrame("utm",utmPose);
//    utmPub.publish(utmPose);

//    geometry_msgs::Pose2D mapPose;
//    CreatePoseMsgForFrame("map",mapPose);
//    mapPub.publish(mapPose);

//    loopRate.sleep();
//  }

    // publish sample data for testing
    int x,y = 0;
    while(ros::ok()) {
      ROS_INFO("Publishing sample data x: %d y: %d", x, y);
      geometry_msgs::Pose2D utmPose;
      CreateSamplePoseMsg(utmPose, x, y);
      utmPub.publish(utmPose);

      // increment x,y coords
      x+=2;
      y+=3;
      if (x > 100) {
          x = 0;
          y = 0;
      }

      loopRate.sleep();
    }

  return 0;
}

