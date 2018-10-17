#include "cameraviewwidget.hpp"
#include "ui_cameraviewwidget.h"

CameraViewWidget::CameraViewWidget(QWidget *parent)
    : QWidget(parent), ui(new Ui::CameraViewWidget) {
  ui->setupUi(this);
}

CameraViewWidget::~CameraViewWidget() { delete ui; }
