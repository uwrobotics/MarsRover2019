#include "maincontrolswidget.hpp"
#include "ui_maincontrolswidget.h"
MainControlsWidget::MainControlsWidget(QWidget *parent)
    : QWidget(parent), ui(new Ui::MainControlsWidget) {
  ui->setupUi(this);
}

MainControlsWidget::~MainControlsWidget() { delete ui; }
void MainControlsWidget::on_Automatic_clicked() {
  ui->label_2->setText("Automatic");
}
void MainControlsWidget::on_Arm_clicked() { ui->label_2->setText("Arm"); }
void MainControlsWidget::on_Science_clicked() {
  ui->label_2->setText("Science");
}
void MainControlsWidget::on_Driving_clicked() {
  ui->label_2->setText("Driving");
}
void MainControlsWidget::on_stop_all_clicked() {
  ui->label_2->setText("NAN");
  ui->velocity->setText("NAN");
  ui->current->setText("NAN");
}
void MainControlsWidget::on_manual_toggled() { ui->state->setText("manual"); }
void MainControlsWidget::on_auto_2_toggled() { ui->state->setText("auto"); }

void MainControlsWidget::current100Callback(can_msgs::FrameConstPtr frame) {
    double current = frame->data[0]/5.0;

    std::stringstream strStream;
    strStream << "CURRENT_100A: " << current/5.0;
    ui->current->setText(QString::fromStdString(strStream.str()));
    //outFile << strStream.str() << std::endl;
    //ROS_DEBUG_STREAM_NAMED("SAFETY", strStream.str().c_str());
}

void MainControlsWidget::Init(ros::NodeHandle &nh) {
  mCurSub = nh.subscribe("/can/safety/current_sensor_100A", 1, &MainControlsWidget::current100Callback, this);
}