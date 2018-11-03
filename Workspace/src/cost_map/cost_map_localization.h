// File name: cost_map_localization.h
// Author: Jordan Juravsky
// Date created: Nov. 3, 2018

#include <opencv2/core/core.hpp>
#include <iostream>

void localization_update(cv::Mat & cost_map, double delta_x, double delta_y, double delta_heading);

void scale_map(cv::Mat & cost_map, double scale_factor);
void translate_map(cv::Mat & cost_map, double delta_x, double delta_y);
void rotate_map(cv::Mat & cost_map, double angle);
