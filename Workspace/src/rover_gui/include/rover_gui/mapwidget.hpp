#ifndef MAPWIDGET_HPP
#define MAPWIDGET_HPP

#include <QString>
#include <QWidget>
#include <QtCore>
#include <QtGui>
#ifndef Q_MOC_RUN
#include <robot_localization/navsat_conversions.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#endif

namespace Ui {
class MapWidget;
}

class MapWidget : public QWidget {
  Q_OBJECT

public:
  explicit MapWidget(QWidget *parent = nullptr);
  ~MapWidget();

  bool Init(ros::NodeHandle &nh);

  void subscriber_callback(const geometry_msgs::Pose2D::ConstPtr &receivedMsg);
  float scaling_function(float x_dist, float y_dist);
  void SetLatLon(double lat, double lon);

private:
  Ui::MapWidget *ui;

  ros::NodeHandle *mpNh;

  QGraphicsScene *scene;
  double longitude;
  double latitude;
  double easting_utm;
  double northing_utm;
  std::string utm_zone, rover_utm_zone;

  // subscribers
  ros::Subscriber mSub;
};

#endif // MAPWIDGET_HPP
