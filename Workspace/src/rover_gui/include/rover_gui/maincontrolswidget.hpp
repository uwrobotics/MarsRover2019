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
private Q_SLOTS:
  void on_Automatic_clicked();
  void on_Arm_clicked();
  void on_Science_clicked();
  void on_Driving_clicked();
  void on_stop_all_clicked();
  void on_manual_toggled();
  void on_auto_2_toggled();

private:
  Ui::MainControlsWidget *ui;
};

#endif // MAINCONTROLSWIDGET_H
