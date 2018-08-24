
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/roverGUI/main_window.hpp"
#include "../include/roverGUI/gui.h"

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
    const sensor_msgs::NavSatFix::ConstPtr &receivedMsg) {

  double rover_lat = receivedMsg->latitude;
  double rover_long = receivedMsg->longitude;
  double rover_easting, rover_northing;

  RobotLocalization::NavsatConversions::LLtoUTM(
      rover_lat, rover_long, rover_northing, rover_easting, rover_utm_zone);

  int pixmap_y = ui.myGraphicsView->height();
  int pixmap_x = ui.myGraphicsView->width();
  QPixmap pix = QPixmap(pixmap_x, pixmap_y);

  QPoint centre; // this is the rover. Reference to mid of pixmap
  centre.setX(pixmap_x / 2);
  centre.setY(pixmap_y / 2);
  double x_dist = (easting_utm - rover_easting);
  double y_dist = (northing_utm - rover_northing);

  QPoint p1;
  p1.setX(centre.x() + x_dist * scaling_function(x_dist, y_dist));
  p1.setY(centre.y() + y_dist * scaling_function(x_dist, y_dist));
  ROS_INFO(" x distance to target %f", x_dist);
  ROS_INFO(" y distance to target %f", y_dist);
  ROS_INFO(" x,y scale factor is (pixels/meter) %f",
           scaling_function(x_dist, y_dist));

  QPainter painter(&pix);

  QPen redPen(Qt::red);
  QPen bluePen(Qt::blue);
  redPen.setWidth(pixmap_x / 100); // scale position markers
  bluePen.setWidth(pixmap_x / 100);

  painter.setPen(redPen);
  painter.drawPoint(p1);
  painter.setPen(bluePen);
  painter.drawPoint(centre);

  // ui.myLabel->setPixmap(pix);
  // scene->addPixmap(blankPix);
  scene->clear();        // didnt work still lagging
  scene->addPixmap(pix); // previous dots remain on screen for some reason.
                         // Need to erase everytime the position is updated
}

float roverGUI::MainWindow::scaling_function(float x_dist, float y_dist) {
  double euclidean_dist = sqrt(pow(x_dist, 2) + pow(y_dist, 2));
  if (euclidean_dist <
      300) {  // determine the threshold of 300 by trial and error
    return 5; // arbitrary
  } else {
    return 1; // arbitrary
  }
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
