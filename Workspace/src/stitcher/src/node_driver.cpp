#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "stitcher_code.cpp"

int main(int argc, char **argv)
{  
  ros::init(argc, argv, "nodeTestDriverPublisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  //cv::Mat img1 = imread("/home/tibi/MarsRover2019/Workspace/src/stitcher/testImages/pic1.jpg", 1);
  //cv::Mat img2 = imread("/home/tibi/MarsRover2019/Workspace/src/stitcher/testImages/pic2.jpg", 1);

  cv::Mat img1 = imread("/home/tibi/MarsRover2019/Workspace/src/stitcher/testImages/img1.jpg", 1);
  cv::Mat img2 = imread("/home/tibi/MarsRover2019/Workspace/src/stitcher/testImages/img2.jpg", 1);
  cv::Mat img3 = imread("/home/tibi/MarsRover2019/Workspace/src/stitcher/testImages/img3.jpg", 1);
  cv::Mat img4 = imread("/home/tibi/MarsRover2019/Workspace/src/stitcher/testImages/img4.jpg", 1);
  
  cv::imshow("view", img1);
  cv::waitKey(30);
  ros::Duration(5).sleep();

  sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img1).toImageMsg();
  sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img2).toImageMsg();
  sensor_msgs::ImagePtr msg3 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img3).toImageMsg();
  sensor_msgs::ImagePtr msg4 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img4).toImageMsg();

  ros::Rate loop_rate(5);

  // send number of images equal to vector size
  vector<sensor_msgs::ImagePtr> msgs;
  msgs.push_back(msg1);
  msgs.push_back(msg2);
  msgs.push_back(msg3);
  msgs.push_back(msg4);
  /*
  cv::imshow("view", cv_bridge::toCvCopy(msgs[0])->image);
  cv::waitKey(30);
  ros::Duration(20).sleep();
  */
  for (int i=0; i < msgs.size(); i++) {
    if (nh.ok()) {
      ROS_INFO("Looped through publish");
      pub.publish(msgs[i]);
      //pub.publish(msg2);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  return 0;
}