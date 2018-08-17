
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/roverGUI/main_window.hpp"
#include "../include/roverGUI/gui.h"
#include <QMessageBox>
#include <QString>
#include <QtCore>
#include <QtGui>
#include <iostream>
#include <robot_localization/navsat_conversions.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace roverGUI {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent) {

  longitude = 0;
  latitude = 0;
  easting_utm = 0;
  northing_utm = 0;

  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to
                    // on_...() callbacks in this class.

  scene = new QGraphicsScene(this);

  ui.myGraphicsView->setScene(scene);
  /*change "myGraphicsView" obj name from designer view
    Right now (aug4) the pixMap size and graphics view size is hardcoded.
    This means that a scoll bar appears when the pixmap is bigger than the
    graphics view.
    It would be good to have the pixmap scale dynamically with the
    graphicsview/window resizing
*/

  QObject::connect(
      ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp,
      SLOT(aboutQt())); // qApp is a global variable for the application
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

} // namespace roverGUI

void roverGUI::MainWindow::subscriber_callback(
    const nav_msgs::Odometry::ConstPtr &receivedMsg) {

  float xval = receivedMsg->pose.pose.position.x;
  float yval = receivedMsg->pose.pose.position.y;

  QPixmap pix = QPixmap(300, 300);
  // QPixmap blankPix = QPixmap(300, 300); //didn't work, still lagging

  QPoint p1;
  p1.setX(xval * 40 + 100);
  p1.setY(yval * 40 + 100);
  ROS_INFO(" x value %d", p1.x());
  ROS_INFO(" y value %d", p1.y());
  QPoint p2;
  p2.setX(100);
  p2.setY(100);

  QPainter painter(&pix);

  QPen redPen(Qt::red);
  QPen bluePen(Qt::blue);
  redPen.setWidth(5);
  bluePen.setWidth(5);

  painter.setPen(redPen);
  painter.drawPoint(p1);
  painter.setPen(bluePen);
  painter.drawPoint(p2);
  // ui.myLabel->setPixmap(pix);
  // scene->addPixmap(blankPix);
  scene->clear();        // didnt work still lagging
  scene->addPixmap(pix); // previous dots remain on screen for some reason.
                         // Need to erase everytime the position is updated
}

void roverGUI::MainWindow::on_longitudeLineEdit_returnPressed() {
  on_latitudeLineEdit_returnPressed();
}

void roverGUI::MainWindow::on_latitudeLineEdit_returnPressed() {
  longitude = (ui.longitudeLineEdit->text()).toDouble(); // reads as a QString
  latitude = (ui.latitudeLineEdit->text()).toDouble();
  RobotLocalization::NavsatConversions::LLtoUTM(
      latitude, longitude, northing_utm, easting_utm, utm_zone);
  ROS_INFO(" you input latitude %f", latitude);
  ROS_INFO(" you input longitude %f", longitude);
  ROS_INFO(" easting %f northing %f", easting_utm, northing_utm);
}
