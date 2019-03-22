// OO cost map class.

#ifndef COST_MAP_H
#define COST_MAP_H

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

class CostMap{
public:
    CostMap(int rows, int columns, double scalar_fill);
    virtual ~CostMap();
    void mix(CostMap &other, cv::Mat& mask);
    cv::Mat& getMap();
    void object_update(cv::Mat &observedMask, cv::Mat &grid,
                       obstacle_detection::obstacleDataArrayConstPtr obstacles);
    void publish(ros::Publisher& pub, double resolution, cv::Point center_point);
    void localization_update(double delta_x, double delta_y,
                             double delta_heading, cv::Point2f centre_point);

private:
    cv::Mat map;
    int rows;
    int columns;
    int current_heading, current_x, current_y;
    int last_heading, last_x, last_y;
    int delta_heading, delta_x, delta_y;
};

#endif //COST_MAP_COST_MAP_H
