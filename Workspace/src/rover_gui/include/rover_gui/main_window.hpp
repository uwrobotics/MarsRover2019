/**
 * @file /include/roverGUI/main_window.hpp
 *
 * @brief Qt based gui for roverGUI.
 *
 * @date November 2010
 **/
#ifndef roverGUI_MAIN_WINDOW_H
#define roverGUI_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include "ui_main_window.h"
#include <QApplication>
#include <QString>
#include <QtCore>
#include <QtGui/QMainWindow>
#include <QtGui>

//#include <ros/ros.h>
#include <robot_localization/navsat_conversions.h>
#include <sensor_msgs/NavSatFix.h>

#include <iostream>
#include <math.h>
#include <sstream>
#include <string>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace roverGUI {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  Ui::MainWindowDesign ui;
  MainWindow(int argc, char **argv, ros::NodeHandle &nh, QWidget *parent = 0);
  ~MainWindow();

  bool Exiting() { return mQuitting; }

protected:
  virtual void closeEvent(QCloseEvent *event) {
    ROS_WARN("quitting window");
    mQuitting = true;
    QMainWindow::closeEvent(event);
  }

private:
  ros::NodeHandle &mNh;

  // hack to get combined ros/qt event loop to exit
  bool mQuitting;
};

} // namespace roverGUI

#endif // roverGUI_MAIN_WINDOW_H
