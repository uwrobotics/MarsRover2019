#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "stitcher_code.cpp"

vector<Mat> img_list;
Mat pano;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("start callback");
  cv_bridge::CvImagePtr cv_ptr;
  
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg);

    img_list.push_back(cv_ptr->image);
    //img_list.push_back(cv_bridge::toCvCopy(msg)->image);
    /*
    cv::imshow("view", cv_ptr->image);
    cv::waitKey(30);
    ros::Duration(20).sleep();
    cv::destroyWindow("view");  */
    ROS_INFO("Pushed img");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    ROS_INFO("error");
  }

  if (img_list.size() == 4) {
    //ROS_INFO("Stitching images...");
    /*
    namedWindow( "Display window", CV_WINDOW_AUTOSIZE );// Create a window for display.
    resizeWindow( "Display window", 400 , 400);
    cv::imshow("Display window", img_list[0]);
    cv::waitKey(0);
    cv::destroyWindow("view"); */
    stitch_images(img_list, false, pano);
    ros::Duration(20).sleep();
  }

  ROS_INFO("finished callback");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  //cv::namedWindow("view");
  //cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  //std::string s = std::to_string(countOne);
  ROS_INFO("cunt");
  ros::spin();
  //cv::destroyWindow("view");
}