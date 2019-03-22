// OO cost map class

#include "include/cost_map/CostMap.h"

#include <iostream>
#include <opencv2/core/core.hpp>
#include "include/cost_map/localization.h"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <obstacle_detection/obstacleDataArray.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <algorithm>
#include <cost_map/Costmap.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Int32.h>

//CostMap::CostMap(int rows, int columns, int scalar_fill): rows{rows}, columns{columns},
// map{cv::Mat cost_map(rows, columns, cv::CV_32FC1, cv::Scalar(0.5))},
// current_heading {0}, current_x{0}, current_y{0}, last_heading {0}, last_x{0},
// last_y{0}, delta_heading{0}, delta_x{0}, delta_y{0} {}

CostMap::CostMap(int rows, int columns, double scalar_fill) :
        map{cv::Mat(rows, columns, cv::CV_32FC1, cv::Scalar(scalar_fill))} {}

CostMap::~CostMap() {}

void CostMap::mix(CostMap &other, cv::Mat &mask) {
    cv::Mat odds_old = map / (1.0 - map);
    cv::Mat odds_new = other / (1.0 - other);
    cv::Mat result = odds_old.mul(odds_new);
    result = result / (result + 1.0);
    result.copyTo(map, mask);
}

void CostMap::object_update(cv::Mat &observedMask, cv::Mat &grid,
                            obstacle_detection::obstacleDataArrayConstPtr obstacles) {
    // fill in open space
    cv::ellipse(map, cv::Point(center_point),
                cv::Size(8.0 / RESOLUTION, 8.0 / RESOLUTION), 90, -45, 45,
                cv::Scalar(0.4), -1);
    cv::ellipse(observedMask, cv::Point(center_point),
                cv::Size(8.0 / RESOLUTION, 8.0 / RESOLUTION), 90, -45, 45,
                cv::Scalar(1), -1);

    for (auto &obstacle : obstacles->obstacles) {
        ROS_INFO("obs center %f, %f", obstacle.x, obstacle.z);
        cv::Point2f obstacle_center(obstacle.x, obstacle.z);
        cv::Point2f cam2obs = obstacle_center;
        cv::Point2f normal(cam2obs.y, -cam2obs.x);
        normal /= cv::norm(normal);
        // ROS_INFO("normal: %f, %f", normal.x, normal.y);
        cv::Point2f pt1 = obstacle_center + normal * obstacle.diameter / 2;
        cv::Point2f pt2 = obstacle_center - normal * obstacle.diameter / 2;
        // ROS_INFO("pt 1: %f, %f", pt1.x, pt1.y);
        // ROS_INFO("pt 2: %f, %f", pt2.x, pt2.y);
        pt1 = world2pix(pt1);
        pt2 = world2pix(pt2);
        // ROS_INFO("pt 1: %f, %f", pt1.x, pt1.y);
        // ROS_INFO("pt 2: %f, %f", pt2.x, pt2.y);
        cv::Point pts[3] = {cv::Point(center_point), cv::Point(pt1),
                            cv::Point(pt2)};
        // ROS_INFO("pts: %d, %d --> %d, %d, --> %d, %d", pts[0].x, pts[0].y,
        cv::fillConvexPoly(map, pts, 3, cv::Scalar(P_MISS));

        cv::fillConvexPoly(observedMask, pts, 3, cv::Scalar(1));
    }
    // fill in obstacles
    for (auto &obstacle : obstacles->obstacles) {
        cv::Point2f obstacle_center(obstacle.x, obstacle.z);
        cv::Point2f cam2obs = obstacle_center;
        float angle = atan2(cam2obs.y, cam2obs.x);
        cv::Point center(world2pix(obstacle_center));
        cv::Size axes(std::max(obstacle.diam_major / RESOLUTION, 1.0),
                      std::max(obstacle.diam_minor / RESOLUTION, 1.0));
        cv::ellipse(map, center, axes, obstacle.cov_angle * 180 / M_PI, 0,
                    360, cv::Scalar(P_HIT), -1);
        cv::ellipse(observedMask, center, axes, obstacle.cov_angle * 180 / M_PI,
                    0,
                    360, cv::Scalar(1), -1);
    }
}

void CostMap::publish(ros::Publisher &pub, double resolution,
                      cv::Point center_point) {
    ROS_INFO("publishing");

    //std::cout << costmap << std::endl;
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


void CostMap::localization_update(double delta_x, double delta_y, double delta_heading,
                                  cv::Point2f centre_point) {

    // Calls localization from localization.cpp
    localization_update(map, translation_x, translation_y,
                        delta_heading, center_point);
}