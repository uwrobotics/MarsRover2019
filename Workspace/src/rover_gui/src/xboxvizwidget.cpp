#include "xboxvizwidget.hpp"
#include "ui_xboxvizwidget.h"

xboxvizwidget::xboxvizwidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::xboxvizwidget)
{
    ui->setupUi(this);
}

xboxvizwidget::~xboxvizwidget()
{
    delete ui;
}
