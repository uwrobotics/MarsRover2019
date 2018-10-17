
/*****************************************************************************
** Includes
*****************************************************************************/

#include "main_window.hpp"
#include "gui.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace roverGUI {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char **argv, ros::NodeHandle &nh,
                       QWidget *parent)
    : QMainWindow(parent), mNh(nh), mQuitting(false) {

  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to
                    // on_...() callbacks in this class.

  // Initialize widgets
  ui.mapWidget->Init(mNh);

  QObject::connect(
      ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp,
      SLOT(aboutQt())); // qApp is a global variable for the application
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

} // namespace roverGUI

// TODO: this needs to be moved elsewhere
void roverGUI::MainWindow::on_longitudeLineEdit_returnPressed() {
  on_latitudeLineEdit_returnPressed();
}

void roverGUI::MainWindow::on_latitudeLineEdit_returnPressed() {
  double longitude =
      (ui.longitudeLineEdit->text()).toDouble(); // reads as a QString
  double latitude = (ui.latitudeLineEdit->text()).toDouble();

  ui.mapWidget->SetLatLon(latitude, longitude);
}
