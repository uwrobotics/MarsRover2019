#include "gimbalwidget.hpp"
#include "ui_gimbalwidget.h"

GimbalWidget::GimbalWidget(QWidget *parent)
    : QWidget(parent), ui(new Ui::GimbalWidget) {
  ui->setupUi(this);
}

GimbalWidget::~GimbalWidget() { delete ui; }
