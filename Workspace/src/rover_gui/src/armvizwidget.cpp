#include "armvizwidget.hpp"
#include "ui_armvizwidget.h"
#include "gui.h"
#include <QGraphicsLineItem>
#include <cmath>
#include <algorithm>
#include<ros/console.h>


Arm::Arm(ArmLink shoulder, ArmLink elbow, ArmLink wrist, Claw claw, TurnTable turnTable, RobotFrame robotFrame,
         QPointF startPos, bool isSideView, std::vector<double> angles, QGraphicsItem *parent) :
QGraphicsItem(parent),
shoulder(shoulder),
elbow(elbow),
wrist(wrist),
claw(claw),
turnTable(turnTable),
robotFrame(robotFrame),
startPos(startPos),
isSideView(isSideView),
angles(angles),
mPen(new QPen()),
mBrush(new QBrush())
{
}

Arm::~Arm() {
 
}

QRectF Arm::boundingRect() const{
  double width = std::max(robotFrame.length, 
                          robotFrame.length/2. + shoulder.length + elbow.length + wrist.length + claw.length);
  double height = robotFrame.offset + turnTable.offset + shoulder.length + elbow.length + wrist.length + claw.length;
  return QRectF(0, 0, width, height);
}

void Arm::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
  painter->setPen(*mPen);
  painter->setBrush(*mBrush);

  if (isSideView){
     DrawJoint(painter, shoulder, startPos);
     QPointF shoulder_end = DrawLink(painter,shoulder, startPos, angles[SHOULDER_PITCH]);

    DrawJoint(painter, elbow, shoulder_end);
    QPointF elbow_end = DrawLink(painter, elbow, shoulder_end, angles[ELBOW_PITCH]);

    DrawJoint(painter, wrist, elbow_end);
    QPointF wrist_end = DrawLink(painter, wrist, elbow_end, angles[WRIST_PITCH]);

    DrawClaw(painter, claw, wrist_end);
    DrawTurnTable(painter, turnTable, startPos);
    DrawRobotFrame(painter, robotFrame, startPos, 2);

      //ROS_INFO("%f %f", lineItem->scenePos().x(), lineItem->scenePos().y());
  } else {

  }
}


QPointF Arm::DrawLink(QPainter *painter, ArmLink armLink, QPointF startPos, double pitch){
  if (isSideView){
    mPen->setColor(armLink.linkColor);
    mPen->setWidthF(armLink.thickness);

    qreal start_x = startPos.x();
    qreal start_y = startPos.y();
    qreal end_x = start_x + armLink.length * std::cos(double(pitch));
    //use minus because qt y axis is flipped comparing to the standard cartesian system
    qreal end_y = start_y - armLink.length * std::sin(double(pitch));

    QLineF line(start_x, start_y, end_x, end_y);
    painter->drawLine(line);
    return line.p2();
  } else{

  }
    
}

void Arm::DrawJoint(QPainter *painter, ArmLink armLink, QPointF startPos){
  mPen->setColor(armLink.jointColor);
  mPen->setWidthF(1);
    
  qreal rad = armLink.jointRad;
  QRadialGradient gradient(startPos, rad);
  gradient.setColorAt(0, Qt::white);
  gradient.setColorAt(1, Qt::black);

  delete mBrush;
  mBrush = new QBrush(gradient);
  mBrush->setColor(armLink.jointColor);
  painter->setBrush(*mBrush);

  qreal top_left_x = startPos.x() - rad;
  qreal top_left_y = startPos.y() - rad;
  qreal width = rad * 2;
  qreal height = rad * 2;

  painter->drawEllipse(top_left_x, top_left_y, width, height);
}

void Arm::DrawClaw(QPainter *painter, Claw claw, QPointF startPos){
  if (isSideView){

  } else {

  }
}
  
void Arm::DrawTurnTable(QPainter *painter, TurnTable turnTable, QPointF startPos){
  if (isSideView){
    mPen->setColor(turnTable.outlineColor);
    mPen->setWidthF(1);

    mBrush->setColor(turnTable.fillColor);
    mBrush->setStyle(Qt::Dense3Pattern);

    qreal start_x = startPos.x() - turnTable.radius;
    qreal start_y = startPos.y();
    qreal end_x = start_x + turnTable.radius;
    qreal end_y = start_y;

    QLineF line(start_x, start_y, end_x, end_y);
    painter->drawLine(line);
  } else{

  }
  
}

void Arm::DrawRobotFrame(QPainter *painter, RobotFrame robotFrame, QPointF startPos, qreal offset){
  if (isSideView){
    mPen->setColor(robotFrame.outlineColor);
    mPen->setWidthF(1);

    mBrush->setColor(robotFrame.fillColor);
    mBrush->setStyle(Qt::Dense5Pattern);

    qreal start_x = startPos.x() - robotFrame.length/2.;
    qreal start_y = startPos.y();
    qreal end_x = start_x + robotFrame.length/2;
    qreal end_y = start_y;

    QLineF line(start_x, start_y, end_x, end_y);
    painter->drawLine(line);

  } else{
    
  }
}



armvizwidget::armvizwidget(QWidget *parent) : 
QWidget(parent), 
ui(new Ui::armvizwidget)
{	
  ui->setupUi(this);

  sideScene = new QGraphicsScene(this);
  topScene = new QGraphicsScene(this);

  ui->armVizSideGraphicsView->setScene(sideScene);
  ui->armVizTopGraphicsView->setScene(topScene);

  std::vector<double> angles = {0, 45/180.0*M_PI, 20/180.0*M_PI, 10/180.0*M_PI, 0, 0};
  sideViewArm = createArm(true, angles);
  topViewArm = createArm(false, angles);

  sideScene->addItem(sideViewArm);
  topScene->addItem(topViewArm);

  ui->armVizSideGraphicsView->show();
  ui->armVizTopGraphicsView->show();
}

armvizwidget::~armvizwidget() {
    delete ui;
}

bool armvizwidget::Init(ros::NodeHandle &nh) {
  avNh = &nh;
  mPoseSub = nh.subscribe(ARM_POSE_TOPIC, 1, &armvizwidget::ArmPosCallback, this);
}

void armvizwidget::ArmPosCallback(std_msgs::Float64MultiArrayConstPtr armPos) {
	if(!sideViewArm || !topViewArm){
    sideViewArm = createArm(true, armPos->data);
    topViewArm = createArm(false, armPos->data);
    sideScene->addItem(sideViewArm);
    topScene->addItem(topViewArm);
  } else {

  }
}


Arm* armvizwidget::createArm(bool isSideView, std::vector<double> angles){
  ArmLink shoulder = {SHOULDER_LEN, SHOULDER_THICK, SHOULDER_JOINT_RAD, Qt::gray, Qt::red};
  ArmLink elbow = {ELBOW_LEN, ELBOW_THICK, ELBOW_JOINT_RAD, Qt::gray, Qt::red};
  ArmLink wrist = {WRIST_LEN, WRIST_THICK, WRIST_JOINT_RAD, Qt::gray, Qt::red};
  Claw claw = {CLAW_LEN, CLAW_WIDTH, CLAW_THICK, Qt::black};
  TurnTable turnTable = {TURNTABLE_RAD, TURNTABLE_OFFSET, Qt::black, Qt::gray};
  RobotFrame robotFrame = {ROBOT_LEN, ROBOT_WIDTH, FRAME_OFFSET, Qt::black, Qt::gray};

  Arm* arm = new Arm(shoulder, elbow, wrist, claw, turnTable, robotFrame, QPointF(0,0), isSideView, angles);
  return arm;
}