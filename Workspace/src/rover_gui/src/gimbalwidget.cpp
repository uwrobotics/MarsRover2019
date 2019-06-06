#include "gimbalwidget.hpp"
#include "ui_gimbalwidget.h"

GimbalWidget::GimbalWidget(QWidget *parent)
    : QWidget(parent), ui(new Ui::GimbalWidget) {
  ui->setupUi(this);
}

GimbalWidget::~GimbalWidget() { delete ui; }

void GimbalWidget::Init(ros::NodeHandle &nh) {
  m_canPub = nh.advertise<can_msgs::Frame>("/CAN_transmitter", 1);
}

void GimbalWidget::on_buttonLeft_pressed() {
  can_msgs::Frame frame;
  frame.dlc = 1;
  frame.id = 0x100;
  frame.data[0] = 0;
  m_canPub.publish(frame);
}

void GimbalWidget::on_buttonRight_pressed() {
  can_msgs::Frame frame;
  frame.dlc = 1;
  frame.id = 0x100;
  frame.data[0] = 1;
  m_canPub.publish(frame);
}
