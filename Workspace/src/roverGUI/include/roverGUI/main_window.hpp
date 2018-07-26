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

//#include "qnode.hpp"
#include "ui_main_window.h"
#include <QtCore>
#include <QtGui/QMainWindow>

#include <std_msgs/Int32.h> //need to include everything you are using form std_msgs!
#include <std_msgs/String.h>

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
  MainWindow(int argc, char **argv, QWidget *parent = 0);
  ~MainWindow();

  // void ReadSettings(); // Load up qt program settings at startup
  // void WriteSettings(); // Save qt program settings when closing

  // void closeEvent(QCloseEvent *event); // Overloaded function
  void showNoMasterMessage();
  void subscriber_callback(const std_msgs::Int32::ConstPtr &receivedMsg);
  // void showNoButtonTestMessage();

public Q_SLOTS:
  /******************************************
  ** Auto-connections (connectSlotsByName())
  *******************************************/
  // void on_actionAbout_triggered();
  // void on_button_connect_clicked(bool check );
  // void on_checkbox_use_environment_stateChanged(int state);

  /******************************************
  ** Manual connections
  *******************************************/
  // void updateLoggingView(); // no idea why this can't connect automatically

  // private slots:
  // void on_pushButton_clicked();
  void on_superButton_clicked();

//private:
  //QNode qnode;

protected:
  void paintEvent(); //(QPaintEvent *e);
};

} // namespace roverGUI

#endif // roverGUI_MAIN_WINDOW_H
