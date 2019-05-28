#include "autonomycontrolswidget.hpp"
#include "gui.h"
#include "ui_autonomycontrolswidget.h"
#include <robot_localization/navsat_conversions.h>
#include <cctype>
#include <std_msgs/Empty.h>

AutonomyControlsWidget::AutonomyControlsWidget(QWidget *parent)
    : QWidget(parent), ui(new Ui::AutonomyControlsWidget) {
  ui->setupUi(this);

  // connect(ui->enterButton, SIGNAL(clicked()), this,
  // SLOT(on_latLonButtonPressed()));
}

AutonomyControlsWidget::~AutonomyControlsWidget() { delete ui; }

void AutonomyControlsWidget::Init(ros::NodeHandle &nh) {
  mPub = nh.advertise<geometry_msgs::Pose2D>("/goal/utm", 1);
  mPoseSub = nh.subscribe(UTM_POSE_TOPIC, 1,
                          &AutonomyControlsWidget::PoseCallback, this);
  mStopPub = nh.advertise<std_msgs::Empty>("/autonomy/stop", 1);
}

void AutonomyControlsWidget::PoseCallback(
    geometry_msgs::Pose2DConstPtr receivedMsg) {
  mLastPoseUtm = *receivedMsg;
}

static double GpsStringToDouble (std::string coordStr) {
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
      ROS_INFO_STREAM("DMS Format Latitude Entered: " << degrees << " degrees " << mins << " mins " << secs << " secs. Converts to: " << coord);
  }
  return coord;
}

void AutonomyControlsWidget::on_latLonButton_pressed() {
  geometry_msgs::Pose2D msg;
  std::string utm_zone = ""; // dummy var, unused but required

  std::string latStr = ui->latitudeEdit->text().toStdString();
  double latDouble = GpsStringToDouble(latStr);

  std::string longStr = ui->longitudeEdit->text().toStdString();
  double longDouble = GpsStringToDouble(longStr);

  if (latDouble == 200 || longDouble == 200) {
    return;
  }

  RobotLocalization::NavsatConversions::LLtoUTM(
      latDouble, longDouble, msg.y, msg.x, utm_zone);

  ui->distEdit->setText("");
  ui->headingEdit->setText("");
  mPub.publish(msg);
}

void AutonomyControlsWidget::on_distHeadingButton_pressed() {
  geometry_msgs::Pose2D msg;
  double distance = ui->distEdit->text().toDouble();
  double heading = ui->headingEdit->text().toDouble() * M_PI / 180;
  msg.y = mLastPoseUtm.y + distance * sin(heading);
  msg.x = mLastPoseUtm.x + distance * cos(heading);
  ui->latitudeEdit->setText("");
  ui->longitudeEdit->setText("");

  mPub.publish(msg);
}

void AutonomyControlsWidget::on_stopButton_pressed() {
  std_msgs::Empty msg;
  mStopPub.publish(msg);
}