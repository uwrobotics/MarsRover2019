#include "armvizwidget.hpp"
#include "ui_armvizwidget.h"

armvizwidget::armvizwidget(QWidget *parent) : 
QWidget(parent), 
ui(new Ui::armvizwidget) 
{
    ui->setupUi(this);
}

armvizwidget::~armvizwidget() {
    delete ui;
}
