#ifndef CONSOLEWIDGET_HPP
#define CONSOLEWIDGET_HPP

#include <QPlainTextEdit>
#include <QWidget>
#ifndef Q_MOC_RUN
#include <console_message/console_msg.h>
#include <ros/ros.h>
#endif

class ConsoleWidget : public QPlainTextEdit {
  Q_OBJECT

public:
  explicit ConsoleWidget(QWidget *parent = nullptr);
  ~ConsoleWidget();
  void Init(ros::NodeHandle &nh);

private:
  void ConsoleMessageCallback(console_message::console_msgConstPtr msg);

  ros::Subscriber mSub;
};

#endif // CONSOLEWIDGET_HPP
