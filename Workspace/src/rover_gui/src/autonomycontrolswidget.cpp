#include "autonomycontrolswidget.hpp"
#include "ui_autonomycontrolswidget.h"
#include <sensor_msgs/NavSatFix.h>

AutonomyControlsWidget::AutonomyControlsWidget(QWidget *parent)
    : QWidget(parent), ui(new Ui::AutonomyControlsWidget) {
  ui->setupUi(this);

  // connect(ui->enterButton, SIGNAL(clicked()), this,
  // SLOT(on_latLonButtonPressed()));
}

AutonomyControlsWidget::~AutonomyControlsWidget() { delete ui; }

void AutonomyControlsWidget::Init(ros::NodeHandle &nh) {
  mpPub =
      new ros::Publisher(nh.advertise<sensor_msgs::NavSatFix>("/goal/gps", 1));
}

void AutonomyControlsWidget::on_enterButton_pressed() {
  sensor_msgs::NavSatFix msg;
  msg.latitude = ui->latitudeEdit->text().toDouble();
  msg.longitude = ui->longitudeEdit->text().toDouble();
  msg.header.stamp = ros::Time::now();

  mpPub->publish(msg);
}