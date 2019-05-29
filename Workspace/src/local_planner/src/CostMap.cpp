//
// Created by tom on 25/07/18.
//

#include "CostMap.h"

CostMap::CostMap(double threshold, const RobotParams_t* pRobotParams)
 : m_pParams(pRobotParams), m_threshold(threshold), m_bReady(false) {

}
void CostMap::SetCostMap(cost_map::CostmapConstPtr cost_map_msg) {
  cv_bridge::CvImagePtr cv_ptr;
  ROS_INFO("received costmap");
  // Convert from the ROS image message to a CvImage suitable for working with
  // OpenCV for processing
  try {
    // Always copy, returning a mutable CvImage
    // OpenCV expects color images to use BGR channel order.
    cv_ptr = cv_bridge::toCvCopy(cost_map_msg->costmap, cost_map_msg->costmap.encoding);
  } catch (cv_bridge::Exception &e) {
    // if there is an error during conversion, display it
    ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
    return;
  }
  // ROS_INFO("%f", cv_ptr->image.at<float>(0, 0));
  // Copy the image.data to imageBuf.
  m_costmap = cv_ptr->image;
  m_occGrid = m_costmap > m_threshold;
 
  m_bReady = true;
}

double CostMap::AssessTrajDistance(double turnRad) {

}