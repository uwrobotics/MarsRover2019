#include "autonomycontrolswidget.hpp"
#include "gui.h"
#include "ui_autonomycontrolswidget.h"
#include <robot_localization/navsat_conversions.h>
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

void AutonomyControlsWidget::on_latLonButton_pressed() {
  geometry_msgs::Pose2D msg;
  std::string utm_zone = ""; // dummy var, unused but required
  RobotLocalization::NavsatConversions::LLtoUTM(
      ui->latitudeEdit->text().toDouble(), ui->longitudeEdit->text().toDouble(),
      msg.y, msg.x, utm_zone);

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