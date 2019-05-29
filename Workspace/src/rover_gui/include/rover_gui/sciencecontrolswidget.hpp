#ifndef SCIENCECONTROLSWIDGET_HPP
#define SCIENCECONTROLSWIDGET_HPP

#include <QWidget>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <science_interface/science_status.h>
#endif

namespace Ui {
  class ScienceControlsWidget;
}

class ScienceControlsWidget : public QWidget {
  Q_OBJECT

public:
  explicit ScienceControlsWidget(QWidget *parent = nullptr);

  ~ScienceControlsWidget();

  bool Init(ros::NodeHandle& nh);

private Q_SLOTS:
  void on_augerHeightSlider_valueChanged();
  void on_augerSpeedSlider_valueChanged();
  void on_centrifugeToggle_toggled(bool checked);
  void on_funnelToggle_toggled(bool checked);
  void on_sensorMountToggle_toggled(bool checked);
  void on_centrifugeComboBox_currentIndexChanged(int index);
  void on_elevatorUpButton_pressed();
  void on_elevatorDownButton_pressed();
  void on_elevatorUpButton_released();
  void on_elevatorDownButton_released();


private:
  void ScienceStatusCallback(science_interface::science_statusConstPtr status);
  void SendElevatorPWM(float pwm);
  Ui::ScienceControlsWidget *ui;
  std::vector<std::string> mCentrifugePosNames;

  // Science status subscriber
  ros::Subscriber mScienceStatusSub;

  // Service Clients
  ros::ServiceClient mAugerHeightClient;
  ros::ServiceClient mAugerSpeedClient;
  ros::ServiceClient mCentrifugeOnClient;
  ros::ServiceClient mCentrifugePosClient;
  ros::ServiceClient mFunnelClient;
  ros::ServiceClient mSensorMountClient;

  ros::Publisher mElevatorPwmPub;

  // status
  science_interface::science_statusConstPtr mLastStatus;
};

#endif // SCIENCECONTROLSWIDGET_HPP
