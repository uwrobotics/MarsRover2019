#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/Pose2D.h>
#include <robot_localization/navsat_conversions.h>
#include <std_msgs/Bool.h>


std::vector<geometry_msgs::Pose2D> vecGoals;

ros::Publisher goal_pub;

double GpsStringToDouble (std::string coordStr) {
  size_t size;
  double coord = std::stod(coordStr, &size);
  if(size == coordStr.size() ){ //Decimal Format
      coord = std::stod(coordStr);
    //   ROS_INFO_STREAM("Latitude Entered: " << coord);
  } else { //Degrees, Mins, Secs Format
      size_t dPos = coordStr.find("d");
      size_t minPos = coordStr.find("\'");
      size_t secPos = coordStr.find("\"");
      if (!(isdigit(coordStr[0]) || coordStr[0] == '-') || dPos == std::string::npos || !isdigit(coordStr[dPos+1]) || minPos == std::string::npos 
          || !isdigit(coordStr[minPos+1]) || secPos == std::string::npos) {
        ROS_ERROR_STREAM("Invalid Latitude Format: " << coordStr);
        return -200;
      }

      double degrees = std::stod(coordStr.substr(0, dPos));
      double mins = std::stod(coordStr.substr(dPos + 1, minPos-(dPos + 1)));
      double secs = std::stod(coordStr.substr(minPos + 1, secPos-(minPos+1)));
      if (degrees > 0) {
        coord = degrees + mins/60 + secs/3600;
      } else {
        coord = degrees - mins/60 - secs/3600;
      }
    //   ROS_INFO_STREAM("DMS Format Latitude Entered: " << degrees << " degrees " << mins << " mins " << secs << " secs. Converts to: " << coord);
  }
  return coord;
}


void AutonomyReadyCallback(std_msgs::BoolConstPtr msg) {
    if (msg->data) {
        goal_pub.publish(vecGoals.front());
        vecGoals.erase(vecGoals.begin());
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "autonomy_goal_pub");
    ros::NodeHandle nh;

    goal_pub = nh.advertise<geometry_msgs::Pose2D>("/goal/utm", 1);
    ros::Subscriber sub = nh.subscribe("/autonomy/success", 1, AutonomyReadyCallback);

    std::string input_file = "/home/tom/Desktop/goal_file.txt";
    std::ifstream goal_file(input_file);

    ROS_INFO("Parsing file: %s", input_file.c_str());

    std::string latStr, lonStr;
    while (goal_file >> latStr) {
        goal_file >> lonStr;
        double lat = GpsStringToDouble(latStr);
        double lon = GpsStringToDouble(lonStr);

        if(lat == -200 || lon == -200) {
            ROS_ERROR("Bad coordinate format, please fix file");
            return -1;
        }

        vecGoals.emplace_back();
        std::string utm_zone;
        RobotLocalization::NavsatConversions::LLtoUTM(
            lat, lon, vecGoals.back().y, vecGoals.back().x, utm_zone);

        ROS_INFO("Read goal: lat/lon %s, %s --> lat/lon %f, %f --> utm x: %f, y: %f", latStr.c_str(), lonStr.c_str(), lat, lon, vecGoals.back().x, vecGoals.back().y);

    }

    if (vecGoals.empty()) {
        ROS_ERROR("There were no goals entered");
        return -1;
    }
    ROS_INFO("Sleeping 10 secs before first goal");
    ros::Duration(10).sleep();
    ROS_INFO("Sending first goal");
    goal_pub.publish(vecGoals.front());
    vecGoals.erase(vecGoals.begin());

    ros::Rate loopRate(1);
    while (ros::ok() && !vecGoals.empty()) {
        ros::spinOnce();
        loopRate.sleep();
    }
    if (vecGoals.empty()) {
        ROS_INFO("List of autonomy goals completed");
    }

    return 0;
}