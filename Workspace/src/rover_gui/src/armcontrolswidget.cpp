#include "armcontrolswidget.hpp"
#include "ui_armcontrolswidget.h"

ArmControlsWidget::ArmControlsWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ArmControlsWidget),
    mbFirstCall(true)
{
    ui->setupUi(this);

}

ArmControlsWidget::~ArmControlsWidget()
{
    delete ui;
}

bool ArmControlsWidget::Init(ros::NodeHandle& nh)
{
    mArmModeClient = nh.serviceClient<arm_interface::arm_mode>("/arm_interface/mode_service");

}

void ArmControlsWidget::on_openLoopRadio_clicked()
{
    ROS_INFO("Setting open loop mode");
    SetArmMode(arm_interface::arm_mode::Request::OPEN_LOOP);
}
void ArmControlsWidget::on_velRadio_clicked()
{
    ROS_INFO("Setting velocity ik mode");
    SetArmMode(arm_interface::arm_mode::Request::IK_VEL);
}
void ArmControlsWidget::on_posRadio_clicked()
{
    ROS_INFO("Setting position ik mode");
    SetArmMode(arm_interface::arm_mode::Request::IK_POS);
}

bool ArmControlsWidget::SetArmMode(uint8_t mode)
{
  arm_interface::arm_mode::Request req;
  arm_interface::arm_mode::Response resp;
  req.mode = mode;
  if (!mArmModeClient.exists())
  {
      ROS_INFO("Service doesn't exist");
      mArmModeClient.waitForExistence();
  }
  if (mArmModeClient.call(req, resp))
  {
      if (resp.mode == req.mode)
      {
          ROS_INFO("Successfully set mode");
      }
      else
      {
          ROS_INFO("Failed to set mode");
      }
  }
  else
  {
      ROS_INFO("Failed to contact server");
  }
}