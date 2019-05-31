#ifndef MAPWIDGET_HPP
#define MAPWIDGET_HPP

#include <QString>
#include <QWidget>
#include <QtCore>
#include <QtGui>
#ifndef Q_MOC_RUN
#include <geometry_msgs/Pose2D.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#endif

namespace Ui {
class MapWidget;
}

class Arrow;

class MapWidget : public QWidget {
  Q_OBJECT

public:
  explicit MapWidget(QWidget *parent = nullptr);
  ~MapWidget();

  bool Init(ros::NodeHandle &nh);

  void SetLatLon(double lat, double lon);

private Q_SLOTS:
  void on_baseCalibrateButton_pressed();
private:
  void PoseCallback(geometry_msgs::Pose2DConstPtr receivedMsg);
  void GoalCallback(geometry_msgs::Pose2DConstPtr receivedMsg);
  void BaseAngleCallback(std_msgs::Float64ConstPtr msg);

  Ui::MapWidget *ui;

  ros::NodeHandle *mpNh;

  QGraphicsScene *mScene;
  Arrow *mRoverArrowItem;
  QGraphicsEllipseItem *mGoalItem;
  Arrow *mBaseStationArrowItem;

  double longitude;
  double latitude;
  double easting_utm;
  double northing_utm;
  std::string utm_zone, rover_utm_zone;
  double mScale;

  // subscribers
  ros::Subscriber mPoseSub;
  ros::Subscriber mGoalSub;
  ros::Subscriber mBaseAngleSub;

  // Publishers
  ros::Publisher mBaseCalibratePub;


private Q_SLOTS:
  void ZoomOut();
  void ZoomIn();
};

#endif // MAPWIDGET_HPP
