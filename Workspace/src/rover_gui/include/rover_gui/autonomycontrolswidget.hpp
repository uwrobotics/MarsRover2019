#ifndef AUTONOMYCONTROLSWIDGET_HPP
#define AUTONOMYCONTROLSWIDGET_HPP

#include <QWidget>
#ifndef Q_MOC_RUN
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
  Ui::AutonomyControlsWidget *ui;
  ros::Publisher *mpPub;

public Q_SLOTS:
  void on_enterButton_pressed();
};

#endif // AUTONOMYCONTROLSWIDGET_HPP
