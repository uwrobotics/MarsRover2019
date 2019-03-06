// Jordan Juravsky, Hanz Vora, Ayush Ghosh, Nigel Tims

#include <opencv2/core/core.hpp>
#include <iostream>

// Localization code
#include "include/cost_map/cost_map_localization.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <obstacle_detection/obstacleDataArray.h>
#include <geometry_msgs/Pose2D.h>
#include <cv_bridge/cv_bridge.h>
// commenting out lines that are ROS specific for now.
//#include "ros/ros.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Int32.h>
#include <algorithm>
#include <cost_map/Costmap.h>
#define P_HIT 0.65
#define P_MISS 0.2


void get_localization_info(int & x, int & y, int & heading); // @AYUSH
void get_object_info(); // Look into format of messages published by camera node. @AYUSH
void object_update(cv::Mat & cost_map); // @HANZ

ros::Publisher pub;
//ros::Subscriber locSub;
//ros::Subscriber obstacleSub;



obstacle_detection::obstacleDataArrayConstPtr pObstacleList;
geometry_msgs::Pose2DConstPtr pLastPose;
geometry_msgs::Pose2DConstPtr pNewPose;

void poseCallback(geometry_msgs::Pose2DConstPtr pose)
{
  pNewPose = std::move(pose);
}

void obstacleCallback(obstacle_detection::obstacleDataArrayConstPtr obstacles)
{
  ROS_INFO("received obstacle in callback");
  pObstacleList = std::move(obstacles);
}

void publishCostmap(cv::Mat& costmap, double resolution, cv::Point center_point)
{
  ROS_INFO("publishing");
  //cv::Mat mats[3];
  //cv::split(grid, mats);
  //mats[2] = costmap;
  //cv::Mat merged;
  //cv::merge(mats,3,merged);
  std::cout << costmap << std::endl;
  cv_bridge::CvImage image_msg;
  image_msg.header.stamp = ros::Time::now();
  image_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  image_msg.image = costmap;

  cost_map::Costmap msg;
  image_msg.toImageMsg(msg.costmap);
  msg.resolution = resolution;
  msg.zero_x = center_point.x;
  msg.zero_y = center_point.y;

  pub.publish(msg);
}

void get_translation(geometry_msgs::Pose2D pose1, geometry_msgs::Pose2D pose2, float& translation_x, float& translation_y) {
  float deltax = pose2.x - pose1.x;
  float deltay = pose2.y - pose1.y;
  float cos_t1 = cos(pose1.theta);
  float sin_t1 = sin(pose1.theta);

  translation_x = deltax * (-sin_t1) + deltay * (cos_t1);
  translation_y = deltax * (cos_t1) + deltay * sin_t1;
}

const int RATE = 4;
const float RESOLUTION = 0.25f;
const float X_SIZE = 20.0f;
const float Y_SIZE = 15.0f;
const float Y_MIN = -2.0f;
const int ROWS = (int)(Y_SIZE/RESOLUTION + 1);
const int COLUMNS = (int)(X_SIZE/RESOLUTION + 1);
cv::Point2f center_point( X_SIZE/2/RESOLUTION, -Y_MIN/RESOLUTION);

cv::Point2f world2pix(cv::Point2f pt)
{
  return cv::Point2f((pt.x+X_SIZE/2)/RESOLUTION, (pt.y - Y_MIN)/RESOLUTION);
}

void fill_new_costmap(cv::Mat& newCostmap, cv::Mat& observedMask, cv::Mat& grid, obstacle_detection::obstacleDataArrayConstPtr obstacles)
{
  //cv::Point2f center = ()
  // fill in open space
  cv::ellipse(newCostmap, cv::Point(center_point), cv::Size(8.0/RESOLUTION, 8.0/RESOLUTION), 90, -45, 45, cv::Scalar(0.4), -1);
  cv::ellipse(observedMask, cv::Point(center_point), cv::Size(8.0/RESOLUTION, 8.0/RESOLUTION), 90, -45, 45, cv::Scalar(1), -1);

  for (auto& obstacle : obstacles->obstacles) {
    ROS_INFO("obs center %f, %f", obstacle.x, obstacle.z);
    cv::Point2f obstacle_center(obstacle.x, obstacle.z);
    cv::Point2f cam2obs = obstacle_center;
    cv::Point2f normal(cam2obs.y, -cam2obs.x);
    normal /= cv::norm(normal);
    //ROS_INFO("normal: %f, %f", normal.x, normal.y);
    cv::Point2f pt1 = obstacle_center + normal * obstacle.diameter/2;
    cv::Point2f pt2 = obstacle_center - normal * obstacle.diameter/2;
    //ROS_INFO("pt 1: %f, %f", pt1.x, pt1.y);
    //ROS_INFO("pt 2: %f, %f", pt2.x, pt2.y);
    pt1 = world2pix(pt1);
    pt2 = world2pix(pt2);
    //ROS_INFO("pt 1: %f, %f", pt1.x, pt1.y);
    //ROS_INFO("pt 2: %f, %f", pt2.x, pt2.y);
    cv::Point pts[3] = {cv::Point(center_point), cv::Point(pt1), cv::Point(pt2)};
    //ROS_INFO("pts: %d, %d --> %d, %d, --> %d, %d", pts[0].x, pts[0].y, pts[1].x, pts[1].y, pts[2].x, pts[2].y);
    cv::fillConvexPoly(newCostmap, pts, 3, cv::Scalar(P_MISS));

    cv::fillConvexPoly(observedMask, pts, 3, cv::Scalar(1));

  }
  // fill in obstacles
  for (auto& obstacle : obstacles->obstacles) {
    cv::Point2f obstacle_center(obstacle.x, obstacle.z);
    cv::Point2f cam2obs = obstacle_center;
    float angle = atan2(cam2obs.y, cam2obs.x);
    //cv::Size axes(1, obstacle.diameter/2/RESOLUTION);
    cv::Point center(world2pix(obstacle_center));
    //cv::ellipse(newCostmap, center, axes, angle * 180/M_PI, 0, 360, cv::Scalar(P_HIT), -1);
    //cv::ellipse(observedMask, center, axes, angle * 180/M_PI, 0, 360, cv::Scalar(1), -1);
    cv::Size axes(std::max(obstacle.diam_major/RESOLUTION, 1.0), std::max(obstacle.diam_minor/RESOLUTION, 1.0));
    cv::ellipse(newCostmap, center, axes, obstacle.cov_angle * 180/M_PI, 0, 360, cv::Scalar(P_HIT), -1);
    cv::ellipse(observedMask, center, axes, obstacle.cov_angle * 180/M_PI, 0, 360, cv::Scalar(1), -1);
    //cv::imshow("mask", observedMask);
    //cv::waitKey(0);
  }
}


void mixCostMaps(cv::Mat& main_costmap, cv::Mat& new_costmap, cv::Mat& newmask)
{
  cv::Mat odds_old = main_costmap/(1.0-main_costmap);
  cv::Mat odds_new = new_costmap/(1.0-new_costmap);
  //std::cout << "odds_old = "<< odds_old << std::endl;
  //std::cout << "odds_new = "<< odds_new << std::endl;
  cv::Mat result = odds_old.mul(odds_new);
  //std::cout << "result = "<< result << std::endl;

  result = result/(result + 1.0);
  //std::cout << "result2 = "<< result << std::endl;
  //cv::imshow("mask", result);
  //cv::waitKey(1);
  result.copyTo(main_costmap, newmask);
}


int main(int argc, char** argv)
{




  auto grid = cv::Mat(ROWS, COLUMNS, CV_32FC2);

  for (auto j = 0; j < ROWS; j++) {
    for (auto i = 0; i < COLUMNS; i++) {
      grid.at<cv::Vec2f>(j, i) = cv::Point2f(X_SIZE + i*RESOLUTION, Y_MIN + j*RESOLUTION);
    }
  }


  cv::Mat cost_map(ROWS, COLUMNS, CV_32FC1, cv::Scalar(0.5));


	int current_heading = 0, current_x = 0, current_y = 0;
	int last_heading = 0, last_x = 0, last_y = 0;
	int delta_heading = 0, delta_x = 0, delta_y = 0;

 //cv::Point2f center_point(Y_MIN/RESOLUTION, X_SIZE/2/RESOLUTION);

	// Do we use an array for the object camera stuff? What is the
	// type of the messages published by the camera?

	/* ROS SETUP STUFF @AYUSH HELP. */
     ros::init(argc, argv, "cost_map_node");
     ros:: NodeHandle nh;

     //@Ayush setup publisher/subscriber stuff.
     pub = nh.advertise<cost_map::Costmap>("/autonomy/cost_map", 1);
     ros::Subscriber locSub = nh.subscribe("/localization/pose_utm", 1, poseCallback);
     ros::Subscriber obstacleSub = nh.subscribe("/obstacle_detection/obstacle_list", 1, obstacleCallback);


     ros::Rate loop_rate(RATE);

      while (ros::ok()) {
        ros::spinOnce();
        ROS_INFO("looping");
        if(pNewPose) {
          ROS_INFO("updating");
          if (pLastPose) {
            // Localization update
            float translation_x, translation_y;
            get_translation(*pLastPose, *pNewPose, translation_x, translation_y);

            localization_update(cost_map, translation_x, translation_y, pNewPose->theta - pLastPose->theta,
                                center_point);
          }
          if (pObstacleList) {
            // Obstacle update
            ROS_INFO("doing obstacles");
            cv::Mat new_costmap(cost_map.size(), cost_map.type(), cv::Scalar(0.5));
            cv::Mat mask(cost_map.size(), CV_8UC1, cv::Scalar(0));
            fill_new_costmap(new_costmap, mask, grid, pObstacleList);


            mixCostMaps(cost_map, new_costmap, mask);
            //cv::imshow("costmap", cost_map);
            //cv::waitKey(0);

            pObstacleList = nullptr;

          }

          publishCostmap(cost_map, RESOLUTION, cv::Point(center_point));
          pLastPose = std::move(pNewPose);
          pNewPose = nullptr;
          pObstacleList = nullptr;
        }

        loop_rate.sleep();
      }

    // Replace this with while(ros::ok())
    /*int counter = 0;
	while (counter ++ < 100)
    {
	    get_localization_info(current_x, current_y, current_y);
	    delta_x = current_x - last_x;
	    delta_y = current_y - last_y;
	    delta_heading = current_heading - last_heading;

	    get_object_info();

	    localization_update(cost_map, delta_x, delta_y, delta_heading);

	    object_update(cost_map);

        //loop_rate.sleep();
    }*/

	return EXIT_SUCCESS;
}