
// TEST
#include "gui.h"
#include "main_window.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

  ros::init(argc, argv, "roverGUI");

  ros::NodeHandle nh;

  /*********************
  ** Qt
  **********************/
  QApplication app(argc, argv);
  roverGUI::MainWindow w(argc, argv, nh);

  w.show();

  //  if (!(bool)app.connect(&app, SIGNAL(lastWindowClosed()), &app,
  //                         SLOT(quit()))) {
  //    ROS_WARN("failed1");
  //  };

  ros::Time last_update = ros::Time::now(); // control update frequency

  while (ros::ok() && !w.Exiting()) {

    /*if (!ros::getGlobalCallbackQueue()->empty() &&
        (ros::Time::now() - last_update > ros::Duration(1.5))) {
      last_update = ros::Time::now();*/

    //      ros::getGlobalCallbackQueue()->callOne(ros::WallDuration(0.1));
    //      ros::getGlobalCallbackQueue()->clear();
    ros::spinOnce();
    //}

    // ROS_INFO(" graphics scene hight %d", w.ui.myGraphicsView->height());
    // continually check for Qt user input etc.
    QCoreApplication::processEvents();
  }
  ROS_INFO("exiting");
  // int result = app.exec();

  return 0; // result;
}
