#ifndef CONSOLEWIDGET_HPP
#define CONSOLEWIDGET_HPP

#include <QPlainTextEdit>
#include <QWidget>

class ConsoleWidget : public QPlainTextEdit {
  Q_OBJECT

public:
  explicit ConsoleWidget(QWidget *parent = nullptr);
  ~ConsoleWidget();

private:
};

#endif // CONSOLEWIDGET_HPP
