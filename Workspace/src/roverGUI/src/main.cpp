/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/roverGUI/main_window.hpp"
#include <QApplication>
#include <QtGui>

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
  // ros::start();
  // start();

  ros::NodeHandle nh;

  // ros::Duration(2).sleep();
  ros::Subscriber sub = nh.subscribe(
      TEST_TOPIC, 1, &roverGUI::MainWindow::subscriber_callback, &w);
  ros::spin();

  /* ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
*/

  /*
  QGraphicsView view;
  QGraphicsScene *scene =new QGraphicsScene(&view);
  view.setScene(scene);
  view.resize(200,200);

  QPixmap pix= QPixmap(100,200);

  QPainter painter(&pix);
  QPen paintpen(Qt::red);
  paintpen.setWidth(5);
  painter.setPen(paintpen);
  painter.drawRect(10,10,100,100);

  scene->addPixmap(pix);
  QGraphicsTextItem *text=scene->addText("hey there beautiful");
  view.show();
  */

  int result = app.exec();

  return result;
}
