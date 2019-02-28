// Jordan Juravsky, Hanz Vora, Ayush Ghosh, Nigel Tims

#include <opencv2/core/core.hpp>
#include <iostream>

// Localization code
#include "cost_map_localization.h"

// commenting out lines that are ROS specific for now.
//#include "ros/ros.h"

void get_localization_info(int & x, int & y, int & heading); // @AYUSH
void get_object_info(); // Look into format of messages published by camera node. @AYUSH
void object_update(cv::Mat & cost_map); // @HANZ



int main(int argc, char** argv)
{

    const int ROWS = 50;
    const int COLUMNS = 50;
    const int RATE = 20;

    cv::Mat cost_map(ROWS, COLUMNS, CV_8SC1);

	int current_heading = 0, current_x = 0, current_y = 0;
	int last_heading = 0, last_x = 0, last_y = 0;
	int delta_heading = 0, delta_x = 0, delta_y = 0;


	/* ROS SETUP STUFF @AYUSH
     ros::init(argc, argv, "cost_map_node");
     ros:: NodeHandle nh;

     @Ayush setup publisher/subscriber stuff.

     ros::Rate loop_rate(RATE);
     */


    // Replace this with while(ros::ok())
    /*int counter = 0;
	while (counter ++ < 100)
    {
	    get_localization_info(current_x, current_y, current_y);
	    delta_x = current_x - last_x;
	    delta_y = current_y - last_y;
	    delta_heading = current_heading - last_heading;

	    get_object_info();

	    localization_update(cost_map, delta_x, delta_y, last_heading, current_heading);

	    object_update(cost_map);

        //loop_rate.sleep();
    }*/

	return EXIT_SUCCESS;
}