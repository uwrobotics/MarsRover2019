#include "maincontrolswidget.hpp"
#include "ui_maincontrolswidget.h"
MainControlsWidget::MainControlsWidget(QWidget *parent)
    : QWidget(parent), ui(new Ui::MainControlsWidget) {
  ui->setupUi(this);

}

MainControlsWidget::~MainControlsWidget() { delete ui;}
void MainControlsWidget::on_Automatic_clicked(){
    ui->label_2->setText("Automatic");
}
void MainControlsWidget::on_Arm_clicked(){
    ui->label_2->setText("Arm");
}
void MainControlsWidget::on_Science_clicked(){
    ui->label_2->setText("Science");
}
void MainControlsWidget::on_Driving_clicked(){
    ui->label_2->setText("Driving");
}
void MainControlsWidget::on_stop_all_clicked(){
    ui->label_2->setText("NAN");
    ui->velocity->setText("NAN");
    ui->current->setText("NAN");
}
void MainControlsWidget::on_manual_toggled(){
    ui->state->setText("manual");
}
void MainControlsWidget::on_auto_2_toggled(){
    ui->state->setText("auto");
}
