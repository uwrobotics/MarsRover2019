#ifndef MAPWIDGET_HPP
#define MAPWIDGET_HPP

#include <QString>
#include <QWidget>
#include <QtCore>
#include <QtGui>
#ifndef Q_MOC_RUN
#include <geometry_msgs/Pose2D.h>
#include <robot_localization/navsat_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

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

private:
  void PoseCallback(geometry_msgs::Pose2DConstPtr receivedMsg);
  void GoalCallback(sensor_msgs::NavSatFixConstPtr receivedMsg);

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

private Q_SLOTS:
  void ZoomOut();
  void ZoomIn();
};

#endif // MAPWIDGET_HPP
