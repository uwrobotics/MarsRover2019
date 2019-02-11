#include "armvizwidget.hpp"
#include "ui_armvizwidget.h"
#include "gui.h"
#include <QGraphicsLineItem>
#include <cmath>
#include<ros/console.h>

armvizwidget::armvizwidget(QWidget *parent) : 
QWidget(parent), 
ui(new Ui::armvizwidget),
mPen(new QPen()),
startPos(QPointF(-50, -50))
{	

	shoulder = new ArmLink{SHOULDER_LEN, SHOULDER_THICK, Qt::black};
	elbow = new ArmLink{ELBOW_LEN, ELBOW_THICK, Qt::black};
	wrist = new ArmLink{WRIST_LEN, WRIST_THICK, Qt::black};

    ui->setupUi(this);

    sideScene = new QGraphicsScene(this);
    topScene = new QGraphicsScene(this);

    ui->armVizSideGraphicsView->setScene(sideScene);
    ui->armVizTopGraphicsView->setScene(topScene);

  	ui->armVizSideGraphicsView->show();
  	ui->armVizTopGraphicsView->show();

  	std::vector<double> angles = {0, 45/180.0*M_PI, 20/180.0*M_PI, 10/180.0*M_PI, 0, 0};
  	DrawArmSideView(angles);
}

armvizwidget::~armvizwidget() {
    delete ui;
}

bool armvizwidget::Init(ros::NodeHandle &nh) {
  avNh = &nh;
  mPoseSub = nh.subscribe(ARM_POSE_TOPIC, 1, &armvizwidget::ArmPosCallback, this);
}

void armvizwidget::ArmPosCallback(std_msgs::Float64MultiArrayConstPtr armPos) {
	DrawArmSideView(armPos->data);
	DrawArmTopView(armPos->data);
}


void armvizwidget::DrawArmSideView(std::vector<double> angles){
	QPointF shoulder_end = DrawLink(shoulder, mPen, startPos, angles[SHOULDER_PITCH], sideScene);
	QPointF elbow_end = DrawLink(elbow, mPen, shoulder_end, angles[ELBOW_PITCH], sideScene);
	QPointF wrist_end = DrawLink(wrist, mPen, elbow_end, angles[WRIST_PITCH], sideScene);
}

void armvizwidget::DrawArmTopView(std::vector<double> angles){
	
}

QPointF armvizwidget::DrawLink(armvizwidget::ArmLink *armLink, QPen *mPen, QPointF startPos, double pitch, QGraphicsScene *mScene){
  	mPen->setColor(armLink->mColor);
    mPen->setWidthF(armLink->thickness);

    qreal start_x = startPos.x();
    qreal start_y = startPos.y();
    qreal end_x = start_x + armLink->length * std::cos(double(pitch));
    //use minus because qt y axis is flipped comparing to the standard cartesian system
    qreal end_y = start_y - armLink->length * std::sin(double(pitch));

    QGraphicsLineItem *lineItem = mScene->addLine(start_x,start_y,end_x,end_y, *mPen);
    //ROS_INFO("%f %f", lineItem->scenePos().x(), lineItem->scenePos().y());

    return lineItem->line().p2();
}