// Jordan Juravsky, Hanz Vora, Ayush Ghosh, Nigel Tims

#include <iostream>
#include <opencv2/core/core.hpp>
#include "include/cost_map/CostMap.h"


// Localization code
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
#define P_HIT 0.65
#define P_MISS 0.2


ros::Publisher pub;
// ros::Subscriber locSub
// ros::Subscriber obstacleSub;

obstacle_detection::obstacleDataArrayConstPtr pObstacleList;
geometry_msgs::Pose2DConstPtr pLastPose;
geometry_msgs::Pose2DConstPtr pNewPose;

void poseCallback(geometry_msgs::Pose2DConstPtr pose) {
    pNewPose = std::move(pose);
}

void obstacleCallback(obstacle_detection::obstacleDataArrayConstPtr obstacles) {
    ROS_INFO("received obstacle in callback");
    pObstacleList = std::move(obstacles);
}

void get_translation(geometry_msgs::Pose2D pose1, geometry_msgs::Pose2D pose2,
                     float &translation_x, float &translation_y) {
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
const int ROWS = (int)(Y_SIZE / RESOLUTION + 1);
const int COLUMNS = (int)(X_SIZE / RESOLUTION + 1);
cv::Point2f center_point(X_SIZE / 2 / RESOLUTION, -Y_MIN / RESOLUTION);

cv::Point2f world2pix(cv::Point2f pt) {
    return cv::Point2f((pt.x + X_SIZE / 2) / RESOLUTION,
                       (pt.y - Y_MIN) / RESOLUTION);
}


int main(int argc, char **argv) {

    auto grid = cv::Mat(ROWS, COLUMNS, CV_32FC2);

    for (auto j = 0; j < ROWS; j++) {f
        for (auto i = 0; i < COLUMNS; i++) {
            grid.at<cv::Vec2f>(j, i) =
                    cv::Point2f(X_SIZE + i * RESOLUTION, Y_MIN + j * RESOLUTION);
        }
    }

    //cv::Mat cost_map(ROWS, COLUMNS, CV_32FC1, cv::Scalar(0.5));
    CostMap cost_map(ROWS, COLUMNS);

    // cv::Point2f center_point(Y_MIN/RESOLUTION, X_SIZE/2/RESOLUTION);

    ros::init(argc, argv, "cost_map_node");
    ros::NodeHandle nh;

    //setup publisher/subscriber.
    pub = nh.advertise<cost_map::Costmap>("/autonomy/cost_map", 1);
    ros::Subscriber locSub =
            nh.subscribe("/localization/pose_utm", 1, poseCallback);
    ros::Subscriber obstacleSub =
            nh.subscribe("/obstacle_detection/obstacle_list", 1, obstacleCallback);

    ros::Rate loop_rate(RATE);

    // Main loop
    while (ros::ok()) {
        ros::spinOnce();
        ROS_INFO("looping");
        if (pNewPose) {
            ROS_INFO("updating");
            if (pLastPose) {
                // Localization update
                float translation_x, translation_y;
                get_translation(*pLastPose, *pNewPose, translation_x, translation_y);

//                localization_update(cost_map, translation_x, translation_y,
//                                    pNewPose->theta - pLastPose->theta, center_point);

                cost_map.localization_update(translation_x, translation_y,
                                    pNewPose->theta - pLastPose->theta, center_point);
            }
            if (pObstacleList) {
                // Obstacle update
                ROS_INFO("doing obstacles");

                CostMap new_cost_map(ROWS, COLUMNS, 0.5);
                cv::Mat mask(cost_map.size(), CV_8UC1, cv::Scalar(0));
                new_cost_map.fill_new_costmap(mask, grid, pObstacleList);
                cost_map.mix(new_costmap, mask);

                pObstacleList = nullptr;
            }

            cost_map.publish(pub, RESOLUTION, cv::Point(center_point));
            pLastPose = std::move(pNewPose);
            pNewPose = nullptr;
            pObstacleList = nullptr;
        }

        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}