#include "autonomycontrolswidget.hpp"
#include "gui.h"
#include "ui_autonomycontrolswidget.h"
#include <robot_localization/navsat_conversions.h>
#include <cctype>

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
}

void AutonomyControlsWidget::PoseCallback(
    geometry_msgs::Pose2DConstPtr receivedMsg) {
  mLastPoseUtm = *receivedMsg;
}

void AutonomyControlsWidget::on_latLonButton_pressed() {
  geometry_msgs::Pose2D msg;
  std::string utm_zone = ""; // dummy var, unused but required

  std::string latStr = ui->latitudeEdit->text().toStdString();
  size_t size;
  double latDouble = std::stod(latStr, &size);
  if(size == latStr.size() ){ //Decimal Format
      latDouble = std::stod(latStr);
      ROS_INFO_STREAM("Latitude Entered: " << latDouble);
  } else { //Degrees, Mins, Secs Format
      size_t dPos = latStr.find("d");
      size_t minPos = latStr.find("\'");
      size_t secPos = latStr.find("\"");
      if (!isdigit(latStr[0]) || dPos == std::string::npos || !isdigit(latStr[dPos+1]) || minPos == std::string::npos 
          || !isdigit(latStr[minPos+1]) || secPos == std::string::npos) {
        ROS_ERROR_STREAM("Invalid Latitude Format: " << latStr);
        return;
      }

      double degrees = std::stod(latStr.substr(0, dPos));
      double mins = std::stod(latStr.substr(dPos + 1, minPos-(dPos + 1)));
      double secs = std::stod(latStr.substr(minPos + 1, secPos-(minPos+1)));
      latDouble = degrees + mins/60 + secs/3600;
      ROS_INFO_STREAM("DMS Format Latitude Entered: " << degrees << " degrees " << mins << " mins " << secs << " secs. Converts to: " << latDouble);
  }

  std::string longStr = ui->longitudeEdit->text().toStdString();
  double longDouble = std::stod(longStr, &size);
  if(size == longStr.size() ){ //Decimal Format
      longDouble = std::stod(longStr);
      ROS_INFO_STREAM("Latitude Entered: " << longDouble);
  } else { //Degrees, Mins, Secs Format
      size_t dPos = longStr.find("d");
      size_t minPos = longStr.find("\'");
      size_t secPos = longStr.find("\"");
      if (!isdigit(longStr[0]) || dPos == std::string::npos || !isdigit(longStr[dPos+1]) || minPos == std::string::npos 
          || !isdigit(longStr[minPos+1]) || secPos == std::string::npos) {
        ROS_ERROR_STREAM("Invalid Latitude Format: " << latStr);
        return;
      }

      double degrees = std::stod(longStr.substr(0, dPos));
      double mins = std::stod(longStr.substr(dPos + 1, minPos-(dPos + 1)));
      double secs = std::stod(longStr.substr(minPos + 1, secPos-(minPos+1)));
      longDouble = degrees + mins/60 + secs/3600;
      ROS_INFO_STREAM("DMS Format Longitude Entered: " << degrees << " degrees " << mins << " mins " << secs << " secs. Converts to: " << longDouble);
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
  msg.y = mLastPoseUtm.y + distance * cos(heading);
  msg.x = mLastPoseUtm.x - distance * sin(heading);
  ui->latitudeEdit->setText("");
  ui->longitudeEdit->setText("");

  mPub.publish(msg);
}

double gpsDMSToLongLat(){

}