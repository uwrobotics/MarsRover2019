#include "consolewidget.hpp"
#include <console_message/console_message.h>
ConsoleWidget::ConsoleWidget(QWidget *parent) : QPlainTextEdit(parent) {}

ConsoleWidget::~ConsoleWidget() {}

void ConsoleWidget::Init(ros::NodeHandle &nh) {
  mSub = nh.subscribe("/console_msgs", 100,
                      &ConsoleWidget::ConsoleMessageCallback, this);
}

void ConsoleWidget::ConsoleMessageCallback(
    console_message::console_msgConstPtr msg) {
  std::string strLevel;
  QTextCharFormat tf = this->currentCharFormat();
  switch (msg->level) {
  case ConsoleMessage::INFO:
    strLevel = "[ INFO] ";
    tf.setForeground(QBrush(Qt::green));
    break;
  case ConsoleMessage::WARN:
    strLevel = "[ WARN] ";
    tf.setForeground(QBrush(Qt::darkYellow));
    break;
  case ConsoleMessage::ERROR:
    strLevel = "[ERROR] ";
    tf.setForeground(QBrush(Qt::red));
    break;
  default:
    strLevel = "[UNKWN] ";
  }
  this->setCurrentCharFormat(tf);
  std::string consoleStr =
      strLevel + "[" + std::to_string(msg->time) + "] [" + msg->sender + "] -- " + msg->message;
  this->appendPlainText(QString::fromStdString(consoleStr));
}
