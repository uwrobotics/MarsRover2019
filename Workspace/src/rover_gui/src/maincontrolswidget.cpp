#include "maincontrolswidget.hpp"
#include "ui_maincontrolswidget.h"

MainControlsWidget::MainControlsWidget(QWidget *parent)
    : QWidget(parent), ui(new Ui::MainControlsWidget) {
  ui->setupUi(this);
}

MainControlsWidget::~MainControlsWidget() { delete ui; }
