#ifndef CAMERAVIEWWIDGET_HPP
#define CAMERAVIEWWIDGET_HPP

#include <QWidget>

namespace Ui {
class CameraViewWidget;
}

class CameraViewWidget : public QWidget {
  Q_OBJECT

public:
  explicit CameraViewWidget(QWidget *parent = nullptr);
  ~CameraViewWidget();

private:
  Ui::CameraViewWidget *ui;
};

#endif // CAMERAVIEWWIDGET_HPP
