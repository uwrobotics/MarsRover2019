#ifndef GIMBALWIDGET_HPP
#define GIMBALWIDGET_HPP

#include <QWidget>

namespace Ui {
class GimbalWidget;
}

class GimbalWidget : public QWidget {
  Q_OBJECT

public:
  explicit GimbalWidget(QWidget *parent = nullptr);
  ~GimbalWidget();

private:
  Ui::GimbalWidget *ui;
};

#endif // GIMBALWIDGET_HPP
