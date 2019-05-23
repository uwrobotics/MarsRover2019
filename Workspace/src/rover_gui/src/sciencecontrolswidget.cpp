#include "sciencecontrolswidget.hpp"
#include "ui_sciencecontrolswidget.h"
#include <rover_msgs/SetInt.h>
#include <rover_msgs/SetDouble.h>
#include <rover_msgs/SetFloat.h>
#include <std_srvs/SetBool.h>

ScienceControlsWidget::ScienceControlsWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ScienceControlsWidget)
{
    ui->setupUi(this);

}

ScienceControlsWidget::~ScienceControlsWidget()
{
    delete ui;
}

bool ScienceControlsWidget::Init(ros::NodeHandle& nh)
{
  if (!nh.getParam("/science_interface/centrifuge_pos_names", mCentrifugePosNames))
  {
    ROS_ERROR("Failed to read centrifuge position names");
  }
  for (auto& name : mCentrifugePosNames)
  {
    ui->centrifugeComboBox->addItem(QString::fromStdString(name));
  }

  float auger_min = 0, auger_max = 20;
  if (!nh.getParam("/science_interface/auger_min", auger_min))
  {
    ROS_ERROR("Failed to read auger elevator min");
  }
  if (!nh.getParam("/science_interface/auger_max", auger_max))
  {
    ROS_ERROR("Failed to read auger elevator max");
  }
  ui->augerHeightSlider->setMinimum(auger_min);
  ui->augerHeightSlider->setMaximum(auger_max);

  mScienceStatusSub = nh.subscribe("/science_interface/status", 1, &ScienceControlsWidget::ScienceStatusCallback, this);

  // Set up service clients
  mAugerHeightClient = nh.serviceClient<rover_msgs::SetFloat>("/science_interface/set_auger_height");
  mAugerSpeedClient = nh.serviceClient<rover_msgs::SetFloat>("/science_interface/set_auger_speed");
  mCentrifugeOnClient = nh.serviceClient<std_srvs::SetBool>("/science_interface/set_centrifuge_spinning");
  mCentrifugePosClient = nh.serviceClient<rover_msgs::SetInt>("/science_interface/set_centrifuge_pos");
  mFunnelClient = nh.serviceClient<std_srvs::SetBool>("/science_interface/set_funnel_open");
  mSensorMountClient = nh.serviceClient<std_srvs::SetBool>("/science_interface/set_sensor_mount_deployed");
}

void ScienceControlsWidget::on_augerHeightSlider_valueChanged()
{
  rover_msgs::SetFloat::Request req;
  rover_msgs::SetFloat::Response resp;
  req.data = (float)ui->augerHeightSlider->value();
  if(!mAugerHeightClient.call(req, resp))
  {
    ROS_ERROR("Failed to contact service");
  }
}
void ScienceControlsWidget::on_augerSpeedSlider_valueChanged()
{
  rover_msgs::SetFloat::Request req;
  rover_msgs::SetFloat::Response resp;
  req.data = (float)ui->augerSpeedSlider->value()/10.0;
  if(!mAugerSpeedClient.call(req, resp))
  {
    ROS_ERROR("Failed to contact service");
  }
}
void ScienceControlsWidget::on_centrifugeToggle_toggled(bool checked)
{
  ui->centrifugeToggle->setText((checked)?"On":"Off");
  std_srvs::SetBool::Request req;
  std_srvs::SetBool::Response resp;
  req.data = ui->centrifugeToggle->isChecked();
  if(!mCentrifugeOnClient.call(req, resp))
  {
    ROS_ERROR("Failed to contact service");
    return;
  }
  if (checked)
  {
    ui->centrifugeComboBox->setEnabled(false);
  }
  else if (mLastStatus && !mLastStatus->centrifuge_on)
  {
    ui->centrifugeComboBox->setEnabled(true);
  }

}
void ScienceControlsWidget::on_funnelToggle_toggled(bool checked)
{
  ui->funnelToggle->setText((checked)?"Open":"Closed");
  std_srvs::SetBool::Request req;
  std_srvs::SetBool::Response resp;
  req.data = ui->funnelToggle->isChecked();
  if(!mFunnelClient.call(req, resp))
  {
    ROS_ERROR("Failed to contact service");
  }
}
void ScienceControlsWidget::on_sensorMountToggle_toggled(bool checked)
{
  ui->sensorMountToggle->setText((checked)?"Deployed":"Retracted");
  std_srvs::SetBool::Request req;
  std_srvs::SetBool::Response resp;
  req.data = ui->sensorMountToggle->isChecked();
  if(!mSensorMountClient.call(req, resp))
  {
    ROS_ERROR("Failed to contact service");
  }
}

void ScienceControlsWidget::on_centrifugeComboBox_currentIndexChanged(int index)
{
  rover_msgs::SetInt::Request req;
  rover_msgs::SetInt::Response resp;
req.data = ui->centrifugeComboBox->currentIndex();
  if(!mCentrifugePosClient.call(req, resp))
  {
    ROS_ERROR("Failed to contact service");
  }
}

void ScienceControlsWidget::ScienceStatusCallback(science_interface::science_statusConstPtr status)
{
  ui->labelAugerHeight->setText(QString("%1 cm").arg(status->auger_height));
  ui->labelAugerSpeed->setText(QString("%1 rpm").arg(status->auger_speed));
  ui->labelCentrifugeSpin->setText(QString((status->centrifuge_on)?"On":"Off"));
  ui->labelCentrifugeSpeed->setText(QString("%1 rpm").arg(status->centrifuge_speed));
  ui->labelCentrifugeSpeed->setText(QString::fromStdString(mCentrifugePosNames[status->centrifuge_pos]));
  ui->labelFunnel->setText(QString((status->funnel_open)?"Open":"Closed"));
  ui->labelSensorMount->setText(QString((status->sensors_mount_deployed)?"Deployed":"Retracted"));
  ui->labelTemperature->setText(QString("%1 C").arg(status->temperature));
  ui->labelMoisture->setText(QString("%1 %").arg(status->moisture));
  if (!ui->centrifugeToggle->isChecked() && !status->centrifuge_on)
  {
    ui->centrifugeComboBox->setEnabled(true);
  }
  mLastStatus = std::move(status);
}