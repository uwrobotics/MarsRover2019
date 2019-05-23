#ifndef ARMCONTROLSWIDGET_HPP
#define ARMCONTROLSWIDGET_HPP

#include <QWidget>
#ifndef Q_MOC_RUN
#include <arm_interface/arm_mode.h>
#include <ros/ros.h>
#endif

namespace Ui {
class ArmControlsWidget;
}

class ArmControlsWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ArmControlsWidget(QWidget *parent = nullptr);
    ~ArmControlsWidget();
    bool Init(ros::NodeHandle& nh);

private Q_SLOTS:
  void on_openLoopRadio_clicked();
  void on_posRadio_clicked();
  void on_velRadio_clicked();

private:
  bool SetArmMode(uint8_t mode);


  Ui::ArmControlsWidget *ui;
  ros::ServiceClient mArmModeClient;
  bool mbFirstCall;
};

#endif // ARMCONTROLSWIDGET_HPP
