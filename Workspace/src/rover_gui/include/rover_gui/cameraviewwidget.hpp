#ifndef CAMERAVIEWWIDGET_HPP
#define CAMERAVIEWWIDGET_HPP

#ifndef Q_MOC_RUN

#include <QWidget>
#include "sensor_msgs/Image.h"
#include <ros/ros.h>
#include <cost_map/Costmap.h>

#endif
namespace Ui {
class CameraViewWidget;
}

class CameraViewWidget : public QWidget {
  Q_OBJECT

public:
  explicit CameraViewWidget(QWidget *parent = nullptr);
  ~CameraViewWidget();
  void subscribe(ros::NodeHandle &guiHandle, std::string imageTopic,
                 bool bDepthImg = false, bool bCostmap = false);
  void imageCallback(const sensor_msgs::Image::ConstPtr &msg);
  void costmapCallback(const cost_map::CostmapConstPtr &msg);

private:
  Ui::CameraViewWidget *ui;
  ros::Subscriber sub;
  bool mbDepthImg;
  bool mbCostmap;

  // protected:
  // void resizeEvent(QResizeEvent *);
};

#endif // CAMERAVIEWWIDGET_HPP
