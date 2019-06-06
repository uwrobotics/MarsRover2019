#ifndef GIMBALWIDGET_HPP
#define GIMBALWIDGET_HPP

#include <QWidget>
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <can_msgs/Frame.h>
#endif

namespace Ui {
class GimbalWidget;
}

class GimbalWidget : public QWidget {
  Q_OBJECT

public:
  explicit GimbalWidget(QWidget *parent = nullptr);
  void Init(ros::NodeHandle& nh);
  ~GimbalWidget();

private Q_SLOTS:
  void on_buttonLeft_pressed();
  void on_buttonRight_pressed();

private:
  Ui::GimbalWidget *ui;
  ros::Publisher m_canPub;
};

#endif // GIMBALWIDGET_HPP
