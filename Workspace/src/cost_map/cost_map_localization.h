// File name: cost_map_localization.h
// Author: Jordan Juravsky
// Date created: Nov. 3, 2018

#include <opencv2/core/core.hpp>
#include <iostream>

void localization_update(cv::Mat & cost_map, double delta_x, double delta_y, double last_heading,
                         double current_heading)
