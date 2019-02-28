// File name: cost_map_localization.cpp
// Author: Jordan Juravsky
// Date created: Nov. 3, 2018

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace std;
using namespace cv;

void scale_map(cv::Mat & cost_map, double scale_factor)
{
    cv::multiply(scale_factor, cost_map, cost_map);
}

void translate_map(cv::Mat & cost_map, double shift_left_by, double shift_down_by)
{
    double translation_array[6] = {1,0,shift_left_by,0,1,shift_down_by};
    cv::Mat translation_matrix(2, 3, CV_64F, translation_array);
    cv::warpAffine(cost_map, cost_map, translation_matrix, cost_map.size());
}

void rotate_map(cv::Mat & cost_map, double angle)
{
    int centre_x = cost_map.cols/2, centre_y = cost_map.rows/2;
    cv::Point2f centre_point(centre_x, centre_y);
    cv::Mat rotation_matrix = cv::getRotationMatrix2D(centre_point, angle, 1);
    cv::warpAffine(cost_map, cost_map, rotation_matrix, cost_map.size());
}

void calculate_translations(double delta_x, double delta_y, double last_heading,
                            double current_heading, double & x_translation, double & y_translation){


    double alpha = atan2(-delta_x, delta_y);
    double beta = alpha - last_heading;
    double d = sqrt( pow(delta_x, 2) + pow(delta_y, 2) );

    x_translation = d * sin(beta);
    y_translation = d * cos(beta);

}

void localization_update(cv::Mat & cost_map, double delta_x, double delta_y, double last_heading,
                         double current_heading)
{

    // Following the math derived in meetings.
    double x_translation = 0, y_translation = 0;
    calculate_translations(delta_x, delta_y, last_heading, current_heading,
                           x_translation, y_translation);


    double delta_heading = current_heading - last_heading;


    translate_map(cost_map, x_translation, y_translation);
    rotate_map(cost_map, delta_heading);
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