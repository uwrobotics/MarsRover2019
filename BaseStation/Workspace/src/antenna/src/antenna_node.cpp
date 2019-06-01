#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h> 
#include <std_msgs/Float64.h> 
#include <serial/serial.h> 
#include <string> 
#include <algorithm> 
#include <wiringPi.h> 
#include <softPwm.h>

const uint8_t IN_A_PIN = 6;
const uint8_t IN_B_PIN = 3;
const uint8_t MOTOR_PWM_PIN = 2; 

const uint16_t NUM_SAMPLES = 3;
const uint16_t GEAR_REDUCTION = 40; //40:1 reduction 
const double MIN_ANGLE = -180; //CW
const double MAX_ANGLE = 180; //CCW
const double ANGLE_TOL = 1;

sensor_msgs::NavSatFix avgBasestationFix;
sensor_msgs::NavSatFix avgRoverFix;
:std_msgs::Bool calibrated;

void rover_gps_callback(const sensor_msgs::NavSatFixConstPtr newRoverFix) {
  avgRoverFix.longitude -= avgRoverFix.longitude/NUM_SAMPLES;
  avgRoverFix.longitude += newRoverFix->longitude/NUM_SAMPLES;

  avgRoverFix.latitude -= avgRoverFix.latitude/NUM_SAMPLES;
  avgRoverFix.latitude += newRoverFix->latitude/NUM_SAMPLES;
}

void basestation_gps_callback(const sensor_msgs::NavSatFixConstPtr newBasestationFix) {
  avgBasestationFix.longitude -= avgBasestationFix.longitude/NUM_SAMPLES;
  avgBasestationFix.longitude += newBasestationFix->longitude/NUM_SAMPLES;

  avgBasestationFix.latitude -= avgBasestationFix.latitude/NUM_SAMPLES;
  avgBasestationFix.latitude += newBasestationFix->latitude/NUM_SAMPLES;
}

void calibrated_callback(const std_msgs::Bool calibrationFlag) {
  calibrated.data = calibrationFlag.data;
}

double readAbsoluteEncoder (serial::Serial &serialConnection) {
  if(!serialConnection.isOpen()){
    ROS_ERROR("No Serial Connection to Arduino");
    return -1;
  }
  serialConnection.flush();
  std::string arduinoStr = serialConnection.readline();
 // ROS_INFO_STREAM("serial reading" << arduinoStr);
  try {
    return std::stod(arduinoStr);
  } catch (std::exception e) {
    return -1;
  }
}

void writeMotorPositionCommand(double currentAngle, double angleSetpoint) {
  angleSetpoint = std::max(MIN_ANGLE, angleSetpoint);
  angleSetpoint = std::min(MAX_ANGLE, angleSetpoint);

  if (abs(currentAngle - angleSetpoint) < ANGLE_TOL) {
    digitalWrite(IN_A_PIN, LOW);
    digitalWrite(IN_B_PIN, LOW);
    softPwmWrite(MOTOR_PWM_PIN, 0);
    ROS_INFO("No Rotation");
  } else if (currentAngle < angleSetpoint){
    digitalWrite(IN_A_PIN, LOW );
    digitalWrite(IN_B_PIN, HIGH);
    softPwmWrite(MOTOR_PWM_PIN, 1000);
    ROS_INFO("Positive Rotation");
  } else if (currentAngle > angleSetpoint) {
    digitalWrite(IN_A_PIN, HIGH);
    digitalWrite(IN_B_PIN, LOW);
    softPwmWrite(MOTOR_PWM_PIN, 1000);
    ROS_INFO("Negative Rotation");
  } 
}

int main (int argc, char *argv[]) {
  ros::init(argc, argv, "basestation");
  ros::NodeHandle nh;

  ros::Subscriber calibrated_sub = nh.subscribe("calibrated", 10, calibrated_callback);

  ros::Subscriber rover_gps_sub = nh.subscribe("/rover/fix", 10, rover_gps_callback);
  ros::Subscriber basestation_gps_sub = nh.subscribe("fix", 10, basestation_gps_callback);

  ros::Publisher antenna_heading_pub = nh.advertise<std_msgs::Float64>("antenna_angle", 1000);

  //setup serial
  serial::Serial arduinoSerial("/dev/arduino", 115200, serial::Timeout::simpleTimeout(1000));

  //setup GPIO
  wiringPiSetup();
  pinMode(IN_A_PIN, OUTPUT);
  pinMode(IN_B_PIN, OUTPUT);
 // pinMode(MOTOR_PWM_PIN, OUTPUT) ;
  // digitalWrite(IN_A_PIN, LOW);
  softPwmCreate(MOTOR_PWM_PIN, 0, 1000);
  digitalWrite(IN_B_PIN, LOW);
  digitalWrite(MOTOR_PWM_PIN, LOW);

  ros::Rate anglePublishRate(50); 
  std_msgs::Float64 antenna_heading;
  double desired_angle = 0;
  while (ros::ok()) {
    try {
      //get antenna angle from east 
      double reading = readAbsoluteEncoder(arduinoSerial);
      if (reading == -1) {
	writeMotorPositionCommand(antenna_heading.data, desired_angle);
	anglePublishRate.sleep();
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

      double long1Rad = avgBasestationFix.longitude * M_PI /180;
      double long2Rad  = avgRoverFix.longitude * M_PI / 180;
      double lat1Rad = avgBasestationFix.latitude * M_PI / 180;
      double lat2Rad = avgRoverFix.latitude * M_PI / 180;
      
      double y = sin(long2Rad-long1Rad) * cos(lat2Rad);
      double x = cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(long2Rad-long1Rad);
      desired_angle = atan2(y, x)*180 /M_PI;
      desired_angle =-1*( desired_angle-90);
      if(desired_angle > 180){
      	desired_angle -= 360;
      } else if(desired_angle < -180) {
        desired_angle += 360;
      }

      ROS_INFO ("Desired Angle: %f      Current Angle: %f\n", desired_angle, antenna_heading.data);

      writeMotorPositionCommand(antenna_heading.data, desired_angle);
      anglePublishRate.sleep();
    } catch(std::exception e) {
      softPwmWrite(MOTOR_PWM_PIN, 0);
    } 
 }
 
 //Cleanup Wiringpi GPIO
  softPwmWrite(MOTOR_PWM_PIN, 0);
  digitalWrite(IN_A_PIN, LOW);
  digitalWrite(IN_B_PIN, LOW);
}

