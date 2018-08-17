
// TEST
#include "../include/roverGUI/gui.h"
#include "../include/roverGUI/main_window.hpp"
#include <QApplication>
#include <QtGui>

#include <ros/callback_queue.h>
#include <ros/network.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Int32.h> //need to include everything you are using form std_msgs!
#include <std_msgs/String.h>
#include <string>
/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

  /*********************
  ** Qt
  **********************/
  QApplication app(argc, argv);
  roverGUI::MainWindow w(argc, argv);

  w.show();

  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

  ros::init(argc, argv, "roverGUI");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe(
      ODOMETRY_TOPIC, 1, &roverGUI::MainWindow::subscriber_callback, &w);

  ros::Time last_update = ros::Time::now(); // control update frequency

  while (ros::ok()) {

    if (!ros::getGlobalCallbackQueue()->empty() &&
        (ros::Time::now() - last_update > ros::Duration(1.5))) {
      last_update = ros::Time::now();
      ros::getGlobalCallbackQueue()->callOne(ros::WallDuration(0.1));
      ros::getGlobalCallbackQueue()->clear();
    }
    // continually check for Qt user input etc.
    QCoreApplication::processEvents();
  }

  int result = app.exec();

  return result;
}
