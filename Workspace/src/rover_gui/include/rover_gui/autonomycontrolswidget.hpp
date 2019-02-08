#ifndef AUTONOMYCONTROLSWIDGET_HPP
#define AUTONOMYCONTROLSWIDGET_HPP

#include <QWidget>
#ifndef Q_MOC_RUN
#include <geometry_msgs/Pose2D.h>
#include <ros/ros.h>
#endif

namespace Ui {
class AutonomyControlsWidget;
}

class AutonomyControlsWidget : public QWidget {
  Q_OBJECT

public:
  explicit AutonomyControlsWidget(QWidget *parent = nullptr);
  ~AutonomyControlsWidget();

  void Init(ros::NodeHandle &nh);

private:
  void PoseCallback(geometry_msgs::Pose2DConstPtr receivedMsg);
  Ui::AutonomyControlsWidget *ui;
  ros::Publisher mPub;
  ros::Subscriber mPoseSub;
  geometry_msgs::Pose2D mLastPoseUtm;

public Q_SLOTS:
  void on_latLonButton_pressed();
  void on_distHeadingButton_pressed();
};

#endif // AUTONOMYCONTROLSWIDGET_HPP
