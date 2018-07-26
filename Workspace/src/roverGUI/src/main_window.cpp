
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/roverGUI/main_window.hpp"
#include "../include/roverGUI/gui.h"
#include <QMessageBox>
#include <QtGui>
#include <iostream>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace roverGUI {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent){

  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to
                    // on_...() callbacks in this class.
  QObject::connect(
      ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp,
      SLOT(aboutQt())); // qApp is a global variable for the application


}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}


} // namespace roverGUI

void roverGUI::MainWindow::on_superButton_clicked() {
  QMessageBox msgBox;
  msgBox.setText("that tickles");
  msgBox.exec();

  roverGUI::MainWindow::paintEvent(); // update the image whenever you hit the
                                      // button
}
void roverGUI::MainWindow::paintEvent() { //(QPaintEvent *e){
                                          /*
                                              //QPainter painter(ui.roadMap);
                                              QPainter painter(this);
                                              painter.setWindow(QRect(0,0,100,100));
                                              QPen paintpen(Qt::black);
                                              painter.setPen(paintpen);
                                              painter.drawRect(10,10,20,20);
                                          */

  QPixmap pix = QPixmap(300, 300);

  QPoint p1;
  p1.setX(ui.mySlider->value());
  p1.setY(ui.mySlider->value());

  QPoint p2;
  p2.setX(100);
  p2.setY(100);

  QPainter painter(&pix);
  QPen paintpen(Qt::red);
  paintpen.setWidth(5);
  painter.setPen(paintpen);
  painter.drawPoint(p1);
  painter.drawPoint(p2);
  ui.myLabel->setPixmap(pix); // use label to add images (pixmapo in this case)
}
void roverGUI::MainWindow::subscriber_callback(
    const std_msgs::Int32::ConstPtr &receivedMsg) {

  int number = 0;
  if (receivedMsg->data %2==0) {
    number = receivedMsg->data;

    ROS_INFO("%d", receivedMsg->data);
    QPixmap pix = QPixmap(300, 300);

    QPoint p1;
    p1.setX(receivedMsg->data);
    p1.setY(receivedMsg->data);

    QPoint p2;
    p2.setX(100);
    p2.setY(100);

    QPainter painter(&pix);
    QPen paintpen(Qt::red);
    paintpen.setWidth(5);
    painter.setPen(paintpen);
    painter.drawPoint(p1);
    painter.drawPoint(p2);
    ui.myLabel->setPixmap(pix); //previous dots remain on screen for some reason
  }
}
