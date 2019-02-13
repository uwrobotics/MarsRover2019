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
mBrush(new QBrush()),
startPos(QPointF(-50, -50))
{	

	shoulder = new ArmLink{SHOULDER_LEN, SHOULDER_THICK, JOINT_RAD, Qt::gray, Qt::black};
	elbow = new ArmLink{ELBOW_LEN, ELBOW_THICK, JOINT_RAD, Qt::gray, Qt::black};
	wrist = new ArmLink{WRIST_LEN, WRIST_THICK, JOINT_RAD, Qt::gray, Qt::black};

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
	DrawJoint(shoulder, mPen, mBrush, startPos, sideScene);
	QPointF shoulder_end = DrawLink(shoulder, mPen, startPos, angles[SHOULDER_PITCH], sideScene);

	DrawJoint(elbow, mPen, mBrush, shoulder_end, sideScene);
	QPointF elbow_end = DrawLink(elbow, mPen, shoulder_end, angles[ELBOW_PITCH], sideScene);

	DrawJoint(wrist, mPen, mBrush, elbow_end, sideScene);
	QPointF wrist_end = DrawLink(wrist, mPen, elbow_end, angles[WRIST_PITCH], sideScene);

	mPen->setColor(Qt::black);
    mPen->setWidthF(2);

	//DrawTurnTable(startPos, mPen, sideScene, true);
	//DrawRobotFrame(startPos, 2, mPen, sideScene, true);

}

void armvizwidget::DrawArmTopView(std::vector<double> angles){
	
}

QPointF armvizwidget::DrawLink(armvizwidget::ArmLink *armLink, QPen *mPen, QPointF startPos, double pitch, QGraphicsScene *mScene){
  	mPen->setColor(armLink->linkColor);
    mPen->setWidthF(armLink->thickness);

    qreal start_x = startPos.x();
    qreal start_y = startPos.y();
    qreal end_x = start_x + armLink->length * std::cos(double(pitch));
    //use minus because qt y axis is flipped comparing to the standard cartesian system
    qreal end_y = start_y - armLink->length * std::sin(double(pitch));

    QGraphicsLineItem *lineItem = mScene->addLine(start_x, start_y, end_x, end_y, *mPen);
    //ROS_INFO("%f %f", lineItem->scenePos().x(), lineItem->scenePos().y());

    return lineItem->line().p2();
}

void armvizwidget::DrawJoint(ArmLink *armLink, QPen *mPen, QBrush *mBrush, QPointF startPos, QGraphicsScene *mScene){
	mPen->setColor(armLink->jointColor);
    mPen->setWidthF(1);
    
	qreal rad = armLink->jointRad;
	QRadialGradient gradient(startPos, rad);
	gradient.setColorAt(0, Qt::white);
	gradient.setColorAt(1, Qt::black);
	mBrush = new QBrush(gradient);
    mBrush->setColor(armLink->jointColor);

    qreal top_left_x = startPos.x() - rad;
    qreal top_left_y = startPos.y() - rad;
    qreal width = rad * 2;
    qreal height = rad * 2;

    mScene->addEllipse(top_left_x, top_left_y, width, height, *mPen, *mBrush);
}

void armvizwidget::DrawClaw(QPointF startPos, QPen *mPen, QGraphicsScene *mScene, bool isSideView){

}
  
void armvizwidget::DrawTurnTable(QPointF startPos, QPen *mPen, QGraphicsScene *mScene, bool isSideView){
	if (isSideView){
		qreal start_x = startPos.x() - TURNTABLE_RAD;
    	qreal start_y = startPos.y();
    	qreal end_x = start_x + TURNTABLE_RAD;
    	qreal end_y = start_y;

   		mScene->addLine(start_x, start_y, end_x, end_y, *mPen);
	} else{

	}
	
}

void armvizwidget::DrawRobotFrame(QPointF startPos, qreal offset, QPen *mPen, QGraphicsScene *mScene, bool isSideView){
	if (isSideView){
		qreal start_x = startPos.x() - ROBOT_LEN/2.;
    	qreal start_y = startPos.y() + offset;
    	qreal end_x = start_x + ROBOT_LEN/2.;
    	qreal end_y = start_y + offset;

   		mScene->addLine(start_x, start_y, end_x, end_y, *mPen);
   		mScene->addLine(startPos.x(), startPos.y(), start_x, start_y + offset, *mPen);

	} else{
		
	}
}