#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <serial/serial.h>
#include <string>
#include <algorithm>
#include <wiringPi.h>

const uint8_t ENABLE_A_PIN = 15;
const uint8_t ENABLE_B_PIN = 7;
const uint8_t MOTOR_PWM_PIN = 9;

const uint16_t NUM_SAMPLES = 50;
const uint16_t GEAR_REDUCTION = 40; //40:1 reduction 
const double MIN_ANGLE = -360; //CW
const double MAX_ANGLE = 360; //CCW

sensor_msgs::NavSatFix avgBasestationFix;
sensor_msgs::NavSatFix avgRoverFix;
std_msgs::Bool calibrated;

void rover_gps_callback(const sensor_msgs::NavSatFixConstPtr newRoverFix) {
  avgRoverFix.longitude -= avgRoverFix.longitude/NUM_SAMPLES;
  avgRoverFix.longitude += newRoverFix->longitude/NUM_SAMPLES;

  avgRoverFix.latitude -= avgRoverFix.latitude/NUM_SAMPLES;
  avgRoverFix.latitude -= newRoverFix->latitude/NUM_SAMPLES;
}

void basestation_gps_callback(const sensor_msgs::NavSatFixConstPtr newBasestationFix) {
  avgBasestationFix.longitude -= avgBasestationFix.longitude/NUM_SAMPLES;
  avgBasestationFix.longitude += newBasestationFix->longitude/NUM_SAMPLES;

  avgBasestationFix.latitude -= avgBasestationFix.latitude/NUM_SAMPLES;
  avgBasestationFix.latitude -= newBasestationFix->latitude/NUM_SAMPLES;
}

void calibrated_callback(const std_msgs::Bool calibrationFlag) {
  calibrated.data = calibrationFlag.data;
}

double readAbsoluteEncoder (serial::Serial &serialConnection) {
  if(!serialConnection.isOpen()){
    ROS_ERROR("No Serial Connection to Arduino");
    return -1;
  }
  std::string arduinoStr = serialConnection.readline();
  return std::stod(arduinoStr);
}

void writeMotorPositionCommand(double currentHeading, double headingSetpoint) {
  headingSetpoint = std::max(MIN_ANGLE, headingSetpoint);
  headingSetpoint = std::min(MAX_ANGLE, headingSetpoint);

  digitalWrite(MOTOR_PWM_PIN, LOW);
  if (currentHeading < headingSetpoint){
    digitalWrite(ENABLE_A_PIN, HIGH);
    digitalWrite(ENABLE_B_PIN, LOW);
    digitalWrite(MOTOR_PWM_PIN, HIGH);
    ROS_DEBUG("Positive Rotation");
  } else if (currentHeading > headingSetpoint) {
    digitalWrite(ENABLE_A_PIN, LOW);
    digitalWrite(ENABLE_B_PIN, HIGH);
    digitalWrite(MOTOR_PWM_PIN, HIGH);
    ROS_DEBUG("Negative Rotation");
  } else {
    digitalWrite(ENABLE_A_PIN, LOW);
    digitalWrite(ENABLE_B_PIN, LOW)i;
    ROS_DEBUG("No Rotation");
  }
}

int main (int argc, char *argv[]) {
  ros::init(argc, argv, "basestation");
  ros::NodeHandle nh;

  ros::Subscriber calibrated_sub = nh.subscribe("calibrated", 10, calibrated_callback);

  ros::Subscriber rover_gps_sub = nh.subscribe("/rover/fix", 10, rover_gps_callback);
  ros::Subscriber basestation_gps_sub = nh.subscribe("fix", 10, basestation_gps_callback);

  ros::Publisher antenna_heading_pub = nh.advertise<std_msgs::Float64>("heading", 1000);

  //setup serial
  serial::Serial arduinoSerial("/dev/arduino", 115200, serial::Timeout::simpleTimeout(1000));

  //setup GPIO
  wiringPiSetupGpio();
  pinMode(ENABLE_A_PIN, OUTPUT);
  pinMode(ENABLE_B_PIN, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);

  ros::Rate anglePublishRate(50); 
  std_msgs::Float64 antenna_heading;
  while (ros::ok()) {
    //get antenna angle from east 
    double reading = readAbsoluteEncoder(arduinoSerial);
    if (reading = -1) {
      continue;
    }
    antenna_heading.data = reading;
    antenna_heading_pub.publish(antenna_heading);

    //read gps coords of rover and basestation
    ros::spinOnce();
    
    if (!calibrated.data) {
      //rotate to east
      writeMotorPositionCommand(antenna_heading.data, 0);
      ROS_INFO_THROTTLE(2, "Antenna Base Station not calibrated. Sending antenna to 0 degrees from East. Currently at %f degrees. Please face the antenna east and press calibrate on the gui!", antenna_heading.data);
      continue;
    }

    double diff_longitude = avgRoverFix.longitude - avgBasestationFix.longitude;
    double diff_latitude = avgRoverFix.latitude - avgBasestationFix.latitude;
    double desired_angle = atan2(diff_latitude, diff_longitude);

    ROS_INFO ("Rover_long: %f,        Rover_lat: %f", avgRoverFix.longitude, avgRoverFix.latitude);
    ROS_INFO ("Base_long: %f,         Base_lat: %f", avgBasestationFix.longitude, avgBasestationFix.latitude);
    ROS_INFO ("Diff_longitude: %f,    Diff_latitude: %f", diff_longitude, diff_latitude); 
    ROS_INFO ("Desired Angle: %f      Current Angle: %f\n", desired_angle, antenna_heading.data);

    writeMotorPositionCommand(antenna_heading.data, desired_angle);
    
    anglePublishRate.sleep();
  }

}
