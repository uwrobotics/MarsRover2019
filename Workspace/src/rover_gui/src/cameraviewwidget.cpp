#include "cameraviewwidget.hpp"
#include "ui_cameraviewwidget.h"
#include "sensor_msgs/Image.h"

CameraViewWidget::CameraViewWidget(QWidget *parent)
    : QWidget(parent), ui(new Ui::CameraViewWidget) {
  ui->setupUi(this);
}

CameraViewWidget::~CameraViewWidget() { delete ui; }

void CameraViewWidget::subscribe(ros::NodeHandle &guiHandle)
{
	sub = guiHandle.subscribe("cameratopic",1000, &CameraViewWidget::imageCallback, this);
}

void CameraViewWidget::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	QImage temp(&(msg->data[0]), msg->width, msg->height, QImage::Format_RGB888);
	QImage image;
	image = temp;
	ui->label->setPixmap(QPixmap::fromImage(image));
}

