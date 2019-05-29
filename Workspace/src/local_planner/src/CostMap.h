//
// Created by tom on 25/07/18.
//

#ifndef PROJECT_COSTMAPBASE_H
#define PROJECT_COSTMAPBASE_H
#include <cost_map/Costmap.h>
#include <cv_bridge/cv_bridge.h>
#include "RoverParams.h"
#include <ros/ros.h>


class CostMap {
  public:
    CostMap(double threshold, const RobotParams_t* pRobotParams);
    void SetCostMap(cost_map::CostmapConstPtr costmap_msg);
    void SetThreshold(double thresh) {m_threshold = thresh;}
    bool IsReady() {return m_bReady;}
    double AssessTrajDistance(double turnRad);
  private:
    cv::Mat m_costmap;
    cv::Mat m_occGrid;
    double x_res;
    double y_res;
    int height;
    int width;
    double m_threshold;
    const RobotParams_t* m_pParams;
    bool m_bReady;


};

#endif // PROJECT_COSTMAPBASE_H
