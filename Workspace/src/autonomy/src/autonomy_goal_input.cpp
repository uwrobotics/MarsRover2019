//
// Created by tom on 26/05/18.
//

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

sensor_msgs::NavSatFixConstPtr pCurGps = nullptr;

void CurGpsCallback(sensor_msgs::NavSatFixConstPtr pGps) { pCurGps = pGps; }

static inline double radians(double degrees) { return degrees * M_PI / 180; }
static inline double degrees(double radians) { return radians * 180 / M_PI; }

const double Eradius = 6371;

sensor_msgs::NavSatFix CalculateGoal(double bearing, double distance) {

  double CurLon = /*radians(-110.776861);//*/ radians(pCurGps->longitude);
  double CurLat = /*radians(38.389972);//*/ radians(pCurGps->latitude);
  bearing = radians(bearing);
  double DestLat = asin(sin(CurLat) * cos(distance / Eradius) +
                        cos(CurLat) * sin(distance / Eradius) * cos(bearing));
  double DestLon =
      CurLon + atan2(sin(bearing) * sin(distance / Eradius) * cos(CurLat),
                     cos(distance / Eradius) - sin(CurLat) * sin(DestLat));
  DestLon = (DestLon + 3 * M_PI) / (2 * M_PI);
  int i = DestLon;
  DestLon = (DestLon - i) * (2 * M_PI) - M_PI; // normalise to -180..+180º
  DestLon = degrees(DestLon);
  DestLat = degrees(DestLat);

  std::cout << "Lat " << DestLat << ", Lon " << DestLon << std::endl;

  sensor_msgs::NavSatFix msg;
  msg.latitude = DestLat;
  msg.longitude = DestLon;
  msg.header.stamp = ros::Time::now();
  return msg;
}

sensor_msgs::NavSatFix AbsoluteGoal(double lat, double lon) {
  sensor_msgs::NavSatFix msg;
  msg.latitude = lat;
  msg.longitude = lon;
  msg.header.stamp = ros::Time::now();
  return msg;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "autonomy_input_node");
  ros::NodeHandle nh;
  ROS_INFO("autonomy input");

  ros::Subscriber gpsSub = nh.subscribe("/gps/filtered", 1, CurGpsCallback);
  ros::Publisher goalPub =
      nh.advertise<sensor_msgs::NavSatFix>("/autonomy/goal_gps", 1);

  ROS_INFO("autonomy input setup complete");

  while (ros::ok()) {
    ros::spinOnce();
    //    if (pCurGps) {

    std::string mode;
    std::cout << "Lat/Lon(l) or Dist/Bearing(d): ";
    std::cin >> mode;
    if (mode == "d") {
      double dist;
      double degrees;
      std::cout << "enter distance: ";
      std::cin >> dist;

      std::cout << "enter degrees: ";
      std::cin >> degrees;
      ros::spinOnce(); // update to latest current gps;
      goalPub.publish(CalculateGoal(degrees, dist));
    } else if (mode == "l") {
      double lat;
      double lon;
      std::cout << "enter latitude : ";
      std::cin >> lat;

      std::cout << "enter longitude: ";
      std::cin >> lon;
      ros::spinOnce(); // update to latest current gps;
      goalPub.publish(AbsoluteGoal(lat, lon));
    }

    //  }
  }
}
