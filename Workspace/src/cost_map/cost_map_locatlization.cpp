// File name: cost_map_localization.cpp
// Author: Jordan Juravsky
// Date created: Nov. 3, 2018

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace std;
//using namespace cv;
static const double RESOLUTION = 0.5;

void scale_map(cv::Mat & cost_map, double scale_factor)
{
    cv::subtract(cost_map, cv::Scalar(0.5), cost_map);
    cv::multiply(scale_factor, cost_map, cost_map);
    cv::add(cost_map, 0.5, cost_map);
}

void translate_map(cv::Mat & cost_map, double shift_left_by, double shift_down_by)
{
    double translation_array[6] = {1,0,shift_left_by,0,1,shift_down_by};
    cv::Mat translation_matrix(2, 3, CV_64F, translation_array);
    cv::warpAffine(cost_map, cost_map, translation_matrix, cost_map.size());
}

void rotate_map(cv::Mat & cost_map, double angle, cv::Point2f centre_point)
{
    //int centre_x = cost_map.cols/2, centre_y = cost_map.rows/2;
    //cv::Point2f centre_point(centre_x, centre_y);
    cv::Mat rotation_matrix = cv::getRotationMatrix2D(centre_point, angle * 180.0/M_PI, 1);
    cv::warpAffine(cost_map, cost_map, rotation_matrix, cost_map.size());
}


void rotate_and_translate_map(cv::Mat & cost_map, double shift_left_by, double shift_down_by, double angle, cv::Point2f centre_point)
{
    double translation_array[6] = {0,0,shift_left_by,0,0,shift_down_by};
    cv::Mat translation_matrix(2, 3, CV_64F, translation_array);
    cv::Mat rotation_matrix = cv::getRotationMatrix2D(centre_point, angle * 180.0/M_PI, 1);
    rotation_matrix = rotation_matrix + translation_matrix;
    cv::warpAffine(cost_map, cost_map, rotation_matrix, cost_map.size(),cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0.5));
}

void localization_update(cv::Mat & cost_map, double delta_x, double delta_y, double delta_heading, cv::Point2f centre_point)
{
    scale_map(cost_map, 0.9);
    //translate_map(cost_map, delta_x/RESOLUTION, -delta_y/RESOLUTION);
    //rotate_map(cost_map, delta_heading, centre_point);
    rotate_and_translate_map(cost_map, delta_x/RESOLUTION, -delta_y/RESOLUTION, delta_heading, centre_point);
}

// Testing.
//int main(int argc, char ** argv)
//{
//    double array[100] = {0};
//    for (int i = 0; i < 100; i++)
//    {
//        array[i] = i;
//    }
//    cv::Mat m(10, 10, CV_64F, array);
//    cout << m << endl << endl;
//
//    cout << "SCALED:" << endl;
//    scale_map(m, 0.5);
//    cout << m << endl;
//
//    system("PAUSE");
//    return EXIT_SUCCESS;
//}