#ifndef MAINCONTROLSWIDGET_H
#define MAINCONTROLSWIDGET_H

#include <QWidget>

namespace Ui {
class MainControlsWidget;
}

class MainControlsWidget : public QWidget {
  Q_OBJECT

public:
  explicit MainControlsWidget(QWidget *parent = nullptr);
  ~MainControlsWidget();

private:
  Ui::MainControlsWidget *ui;
};

#endif // MAINCONTROLSWIDGET_H
