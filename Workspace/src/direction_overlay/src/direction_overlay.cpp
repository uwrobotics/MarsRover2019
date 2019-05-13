#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose2D.h>
#include <string>
#include <sstream>
#include <iostream>

// Code to add precision to modify the precision of the to_string function
template <typename T> 
std::string to_string_f(const T& val, const int n = 2){
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << val;
    return out.str();
}


using namespace cv;
using namespace std;

// Global variables
float heading = 0;
image_transport::Publisher pubImage;
image_transport::Subscriber subImage;

/* Work in progress 
geometry_msgs::Pose2DConstPtr curr = NULL;

void utmPubCallback(geometry_msgs::Pose2DConstPtr msg) { //Work in progress
	curr = msg; 
}*/
	

void imageCallback(const sensor_msgs::Image::ConstPtr& image){
    ROS_INFO("Left Rectified image received from ZED - Size: %dx%d", image->width, image->height);
    cv_bridge::CvImagePtr cv_image_ptr;
    try{
      cv_image_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Variables for the compass scale
    heading = heading + 1.5; //Incrementing heading for testing
    heading = fmod(heading, 360.0);
    //float heading = 10;
    int lineThickness = 2;
    int scale = 2;
    int tallHeight = round(image->height * 0.06);
    int shortHeight = round(tallHeight * 0.6); 
    int headingSpacing = round(tallHeight / 3.0); // Spacing between heading line and scale lines
    int leftBound = image->width / 2 - scale * lineThickness * 75; // x coordinate of left boundary
    int rightBound = image->width / 2 + scale * lineThickness * 75; // x coordinate of right boundary
    int closestHeading = round(heading / 5.0) * 5; // Determine closest heading marking
    int closestHeadingLoc = image->width / 2 + scale * lineThickness * (closestHeading - round(heading)); // Determine x coord of closest heading marking
    int currHeading = closestHeading; // Variable to keep track of heading on the compass
    int currLoc = closestHeadingLoc; // x coordinate of heading on the compass scale
   
    // Variables for compass scale labels
    string label = to_string_f(heading);
    int fontFace = FONT_HERSHEY_SIMPLEX;
    float fontScale = 1.0;
    Size textSize;
    int baseline = 0;
    int labelSpace = 10 * round(image->height * 0.001); // Spacing between label and compass scale

    // Draw red line to mark the heading and display the exact heading value
    line(cv_image_ptr->image, Point(image->width / 2, image->height - round(image->height*0.8) - headingSpacing), 
         Point(image->width / 2, image->height - round(image->height*0.8) - headingSpacing - tallHeight), 
         CV_RGB(255,0,0), lineThickness, LINE_8, 0);
    // Draw label to indicate the heading of the robot
    textSize = getTextSize(label, fontFace, fontScale, lineThickness, &baseline);
    putText(cv_image_ptr->image, label, Point(image->width / 2 - textSize.width/2, image->height - round(image->height*0.8) - tallHeight - textSize.height - labelSpace), 
            fontFace, fontScale, CV_RGB(255,255,255), lineThickness, LINE_AA, false);

    // Draw the right side of the compass
    for(; currLoc <= rightBound; currHeading = (currHeading + 5) % 360, currLoc += scale * lineThickness * 5){
      if(currHeading % 30 == 0){
        // Draw long line
        line(cv_image_ptr->image, Point(currLoc, image->height - round(image->height*0.8)), 
             Point(currLoc, image->height - round(image->height*0.8) - tallHeight), 
             CV_RGB(255,255,255), lineThickness, LINE_8, 0);
        if(currHeading == 0 || currHeading == 360){
          label = "N"; // North
        }
        else if(currHeading == 90){
          label = "E"; // East
        }
        else if(currHeading == 180){
          label = "S"; // South
        }
        else if(currHeading == 270){
          label = "W"; // West
        }
	else{
          label = to_string(currHeading); //Heading that isn't NESW but is a multiple of 15
        }
        // Print label centered on the line
        textSize = getTextSize(label, fontFace, fontScale, lineThickness, &baseline);
        putText(cv_image_ptr->image, label, Point(currLoc - textSize.width/2, image->height - round(image->height*0.8) + textSize.height + labelSpace), 
                fontFace, fontScale, CV_RGB(255,255,255), lineThickness, LINE_AA, false);
      }
      else{
        // Draw short line
        line(cv_image_ptr->image, Point(currLoc, image->height - round(image->height * 0.8)), 
             Point(currLoc, image->height - round(image->height * 0.8) - shortHeight), 
             CV_RGB(255, 255, 255), lineThickness, LINE_8, 0);
      }
    }
    // Draw the left side of the compass
    for(currHeading = closestHeading, currLoc = closestHeadingLoc; currLoc >= leftBound; currHeading -= 5, currLoc -= scale * lineThickness * 5) {
      if(currHeading < 0){
        currHeading = 360 + currHeading;
      }
      if(currHeading % 30 == 0){
        // Draw long line
        line(cv_image_ptr->image, Point(currLoc, image->height - round(image->height * 0.8)),
             Point(currLoc, image->height - round(image->height * 0.8) - tallHeight), 
             CV_RGB(255, 255, 255), lineThickness, LINE_8, 0);
        if(currHeading == 0 || currHeading == 360){
          label = "N"; // North
        }
        else if(currHeading == 90){
          label = "E"; // East
        }
        else if(currHeading == 180){
          label = "S"; // South
        }
        else if(currHeading == 270){
          label = "W"; // West
        }
	else{
          label = to_string(currHeading); //Heading that isn't NESW but is a multiple of 15
        }
        // Print label centered on the line
        textSize = getTextSize(label, fontFace, fontScale, lineThickness, &baseline);
        putText(cv_image_ptr->image, label, Point(currLoc - textSize.width/2, image-> height - round(image->height*0.8) + textSize.height + labelSpace), 
                fontFace, fontScale, CV_RGB(255,255,255), lineThickness, LINE_AA, false);
      }
      else{
        // Draw short line
        line(cv_image_ptr->image, Point(currLoc, image->height - round(image->height * 0.8)), 
             Point(currLoc, image->height - round(image->height * 0.8) - shortHeight), 
             CV_RGB(255, 255, 255), lineThickness, LINE_8, 0);
      }
    }
      
    // Output modified video stream
    pubImage.publish(cv_image_ptr->toImageMsg());
}

int main(int argc, char** argv){
    ros::init(argc, argv, "direction_overlay");
    
    ros::NodeHandle n;
    image_transport::ImageTransport it_(n);

    // Direction variables - work in progress -------------------------------------------
    //latA, longA, latB, longB, X, Y,

    //double theta = 0;
	
    //double latA = ; 
    //double longA = ; 
    //double latB = ; 
    //double longB = ;

    //double X = cos(latB) * sin(longB-longA);
    //double Y = cos(latA) * sin(latB) - sin(latA) * cos(latB) * cos(longB-longA);

    //this is the bearing in degrees, 0 is north, 90 is east, 180 is south, 270 is west
    //double theta = atan2(X, Y);

    //ros::Subscriber utmSub = n.subscribe<geometry_msgs::Pose2D>("/localization/pose_utm", 1 utmPubCallback);
    // ----------------------------------------------------------------------------------

    //subImage = it_.subscribe("/zed/left/image_rect_color", 1, imageCallback); //zed camera feed
    subImage = it_.subscribe("/camera/image_raw", 1, imageCallback); //testing with pointgrey camera
    pubImage = it_.advertise("/direction_overlay/image", 1);
    
    ros::spin();

    return 0;
}
