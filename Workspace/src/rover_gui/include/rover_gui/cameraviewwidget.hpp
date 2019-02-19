#ifndef CAMERAVIEWWIDGET_HPP
#define CAMERAVIEWWIDGET_HPP

#ifndef Q_MOC_RUN

	#include <QWidget>
	#include "sensor_msgs/Image.h"
	#include <ros/ros.h>

#endif
namespace Ui {
class CameraViewWidget;
}

class CameraViewWidget : public QWidget {
  Q_OBJECT

public:
  explicit CameraViewWidget(QWidget *parent = nullptr);
  ~CameraViewWidget();
  void subscribe(ros::NodeHandle &guiHandle);
  void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

private:
  Ui::CameraViewWidget *ui;
  ros::Subscriber sub;
};

#endif // CAMERAVIEWWIDGET_HPP
