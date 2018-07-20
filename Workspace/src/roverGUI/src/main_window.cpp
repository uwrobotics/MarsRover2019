
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/roverGUI/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace roverGUI {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
        : QMainWindow(parent)
        , qnode(argc,argv)
{


        ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    //qnode.init();
    /*ReadSettings();
        setWindowIcon(QIcon(":/images/icon.png"));
        ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
*/
        /*********************
        ** Logging
        **********************/


    /* ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
*/


    /*********************
    ** Auto Start
    **********************/
   /* if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }*/
  //couldnt get graphicsview embedded in main window to work
    /* QGraphicsScene *sceneEmbedded =new QGraphicsScene(&ui.myGraphicsView);
     ui.myGraphicsView->setScene(sceneEmbedded);
     ui.myGraphicsView->resize(200,200);
     QPixmap pix= QPixmap(200,200);

     QPainter painter(&pix);
     QPen paintpen(Qt::red);
     paintpen.setWidth(5);
     painter.setPen(paintpen);
     painter.drawRect(10,10,100,100);
     sceneEmbedded->addPixmap(pix);
*/
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

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */
/*
void MainWindow::on_button_connect_clicked(bool check ) {
        if ( ui.checkbox_use_environment->isChecked() ) {
                if ( !qnode.init() ) {
                        showNoMasterMessage();
                } else {
                        ui.button_connect->setEnabled(false);
                }
        } else {
                if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
                                   ui.line_edit_host->text().toStdString()) ) {
                        showNoMasterMessage();
                } else {
                        ui.button_connect->setEnabled(false);
                        ui.line_edit_master->setReadOnly(true);
                        ui.line_edit_host->setReadOnly(true);
                        ui.line_edit_topic->setReadOnly(true);
                }
        }
}
*/
/*
void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
        bool enabled;
        if ( state == 0 ) {
                enabled = true;
        } else {
                enabled = false;
        }
        ui.line_edit_master->setEnabled(enabled);
        ui.line_edit_host->setEnabled(enabled);
        //ui.line_edit_topic->setEnabled(enabled);
}
*/
/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
/*
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}*/

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/
/*
void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
} */

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/
/*
void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "roverGUI");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
        ui.line_edit_master->setEnabled(false);
        ui.line_edit_host->setEnabled(false);
        //ui.line_edit_topic->setEnabled(false);
    }
}
*/
/*
void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "roverGUI");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}
*/
/*
void MainWindow::closeEvent(QCloseEvent *event)
{
        WriteSettings();
        QMainWindow::closeEvent(event);
}
*/
}  // namespace roverGUI



void roverGUI::MainWindow::on_superButton_clicked()
{
    QMessageBox msgBox;
    msgBox.setText("that tickles");
    msgBox.exec();

    roverGUI::MainWindow::paintEvent(); //update the image whenever you hit the button



}
void roverGUI::MainWindow::paintEvent( ){//(QPaintEvent *e){
/*
    //QPainter painter(ui.roadMap);
    QPainter painter(this);
    painter.setWindow(QRect(0,0,100,100));
    QPen paintpen(Qt::black);
    painter.setPen(paintpen);
    painter.drawRect(10,10,20,20);
*/

    QPixmap pix= QPixmap(300,300);

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
    ui.myLabel->setPixmap(pix); //use label to add images (pixmapo in this case)
}
void roverGUI::MainWindow:: subscriber_callback(const std_msgs::Int32::ConstPtr& receivedMsg){

    int number=0;
    if(receivedMsg->data%4==0){
        number = receivedMsg->data;

        ROS_INFO("Divisible by 4"); QPixmap pix= QPixmap(300,300);

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
        ui.myLabel->setPixmap(pix);

}
}




