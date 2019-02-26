#include "armvizwidget.hpp"
#include "ui_armvizwidget.h"
#include "gui.h"
#include <QRectF>
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
mAngles(angles),
mPen(new QPen()),
mBrush(new QBrush())
{
}

Arm::~Arm() {
  delete mPen;
  delete mBrush;
}

void Arm::setAngles(std::vector<double> angles){
  mAngles.resize(NUM_ANGLES);
  if (angles.size() >= NUM_ANGLES){
//    for (int i=0; i<NUM_ANGLES; i++){
//        mAngles[i] = angles[i];
//      }
    mAngles[TURNTABLE_YAW] = angles[TURNTABLE_YAW] * M_PI/180;
    mAngles[WRIST_ROLL] = angles[WRIST_ROLL] * M_PI/180;
    mAngles[CLAW_CLOSURE_ANGLE] = angles[CLAW_CLOSURE_ANGLE] * M_PI/180;
    mAngles[SHOULDER_PITCH] = angles[SHOULDER_PITCH] * M_PI/180;
    mAngles[ELBOW_PITCH] = mAngles[SHOULDER_PITCH] + angles[ELBOW_PITCH] * M_PI/180;
    mAngles[WRIST_PITCH] = mAngles[ELBOW_PITCH] + angles[WRIST_PITCH] * M_PI/180;
  }
}

QRectF Arm::boundingRect() const{
  double penWidthPadding = 5; 
  double width = std::max(robotFrame.length, 
                          robotFrame.length/2. + shoulder.length + elbow.length + wrist.length + claw.length)
                           + startPos.x() + penWidthPadding;
  double height = robotFrame.offset + turnTable.offset + shoulder.length + elbow.length + wrist.length + claw.length
                + startPos.y() + penWidthPadding;
  double top_left_x = - width/2., top_left_y = -height/2.;
  return QRectF(top_left_x, top_left_y, width, height);
}

void Arm::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
  painter->setRenderHint(QPainter::Antialiasing);
  
  if (isSideView){
    // draw in reverse order, part that needs to appear on topper must be drawn later
    DrawRobotFrame(painter, robotFrame, startPos, 2);
    DrawTurnTable(painter, turnTable, startPos);

    DrawJoint(painter, shoulder, startPos);
    QPointF shoulder_end = DrawLink(painter,shoulder, startPos, mAngles[SHOULDER_PITCH]);

    DrawJoint(painter, elbow, shoulder_end);
    QPointF elbow_end = DrawLink(painter, elbow, shoulder_end, mAngles[ELBOW_PITCH]);

    DrawJoint(painter, wrist, elbow_end);
    QPointF wrist_end = DrawLink(painter, wrist, elbow_end, mAngles[WRIST_PITCH]);

    DrawClaw(painter, claw, wrist_end);
    
  } else {
    DrawRobotFrame(painter, robotFrame, startPos, 2);
    DrawTurnTable(painter, turnTable, startPos);

    DrawJoint(painter, shoulder, startPos);
    QPointF shoulder_end = DrawLink(painter,shoulder, startPos, mAngles[TURNTABLE_YAW]);

    DrawJoint(painter, elbow, shoulder_end);
    QPointF elbow_end = DrawLink(painter, elbow, shoulder_end, mAngles[TURNTABLE_YAW]);

    DrawJoint(painter, wrist, elbow_end);
    QPointF wrist_end = DrawLink(painter, wrist, elbow_end, mAngles[TURNTABLE_YAW]);

    DrawClaw(painter, claw, wrist_end);
  }
  
}


QPointF Arm::DrawLink(QPainter *painter, ArmLink armLink, QPointF startPos, double angle){
  mPen->setColor(armLink.linkColor);
  mPen->setWidthF(armLink.thickness);
  painter->setPen(*mPen);
  painter->setBrush(*mBrush);

  qreal start_x = startPos.x(), start_y = startPos.y();
  qreal end_x, end_y;

  if (isSideView){
    double pitch = angle;
    end_x = start_x + armLink.length * std::cos(double(pitch));
    //use minus because qt y axis is flipped comparing to the standard cartesian system
    end_y = start_y - armLink.length * std::sin(double(pitch));
  } else{
    double yaw = angle;
    end_x = start_x - armLink.length * std::sin(double(yaw));
    end_y = start_y - armLink.length * std::cos(double(yaw));
  }

  QLineF line(start_x, start_y, end_x, end_y);
  painter->drawLine(line);
  return line.p2();
}

void Arm::DrawJoint(QPainter *painter, ArmLink armLink, QPointF startPos){
  mPen->setColor(armLink.jointColor);
  mPen->setWidthF(1);
    
  qreal rad = armLink.jointRad;
  QRadialGradient gradient(startPos, rad);
  gradient.setColorAt(0, armLink.jointColor);
  gradient.setColorAt(1, Qt::white);

  //to set gradient as QRadialGradient, must create new QBrush
  delete mBrush;
  mBrush = new QBrush(gradient);
  mBrush->setColor(armLink.jointColor);
  painter->setBrush(*mBrush);

  painter->setPen(*mPen);
  painter->setBrush(*mBrush);

  qreal top_left_x = startPos.x() - rad;
  qreal top_left_y = startPos.y() - rad;
  qreal width = rad * 2;
  qreal height = rad * 2;

  painter->drawEllipse(top_left_x, top_left_y, width, height);

  //resets gradient
  delete mBrush;
  mBrush = new QBrush();
}

void Arm::DrawClaw(QPainter *painter, Claw claw, QPointF startPos){
  if (isSideView){

  } else {

  }
}
  
void Arm::DrawTurnTable(QPainter *painter, TurnTable turnTable, QPointF startPos){
  mPen->setColor(turnTable.outlineColor);
  mPen->setWidthF(1);

  mBrush->setColor(turnTable.fillColor);
  mBrush->setStyle(Qt::Dense3Pattern);

  painter->setPen(*mPen);
  painter->setBrush(*mBrush);

  if (isSideView){
    qreal start_x = startPos.x() - turnTable.radius;
    qreal start_y = startPos.y() + turnTable.offset;
    qreal end_x = startPos.x() + turnTable.radius;
    qreal end_y = start_y;

    painter->drawLine(start_x, start_y, end_x, end_y);
  } else{
    qreal rad = turnTable.radius;
    qreal top_left_x = startPos.x() - rad;
    qreal top_left_y = startPos.y() - rad;
    qreal width = rad * 2;
    qreal height = rad * 2;
    painter->drawEllipse(top_left_x, top_left_y, width, height);
  }
  
}

void Arm::DrawRobotFrame(QPainter *painter, RobotFrame robotFrame, QPointF startPos, qreal offset){
  mPen->setColor(robotFrame.outlineColor);
  mPen->setWidthF(1);

  mBrush->setColor(robotFrame.fillColor);
  mBrush->setStyle(Qt::Dense6Pattern);

  painter->setPen(*mPen);
  painter->setBrush(*mBrush);

  if (isSideView){
    qreal start_x = startPos.x() - robotFrame.length/2.;
    qreal start_y = startPos.y() + turnTable.offset + robotFrame.offset;
    qreal end_x = startPos.x() + robotFrame.length/2;
    qreal end_y = start_y;

    painter->drawLine(start_x, start_y, end_x, end_y);

  } else{
    qreal top_left_x = startPos.x() - robotFrame.width/2.;
    qreal top_left_y = startPos.y() - robotFrame.length/2.;
    qreal width = robotFrame.width;
    qreal height = robotFrame.length;
    painter->drawRect(top_left_x, top_left_y, width, height);
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

  std::vector<double> angles = {30/180.0*M_PI, 45/180.0*M_PI, 20/180.0*M_PI, 10/180.0*M_PI, 0, 0};
  sideViewArm = createArm(true, angles);
  topViewArm = createArm(false, angles);

  sideScene->addItem(sideViewArm);
  topScene->addItem(topViewArm);

  ui->armVizSideGraphicsView->show();
  ui->armVizTopGraphicsView->show();

  ui->armVizSideGraphicsView->setSceneRect(sideScene->itemsBoundingRect());
  ui->armVizTopGraphicsView->setSceneRect(topScene->itemsBoundingRect());

  sideViewArm->setPos(-10, 35);
  topViewArm->setPos(0, 15);

  // ROS_INFO("%f %f %f %f", sideViewArm->scenePos().x(), sideViewArm->scenePos().y(), topViewArm->pos().x(), topViewArm->pos().y());

}

armvizwidget::~armvizwidget() {
    delete ui;
}

bool armvizwidget::Init(ros::NodeHandle &nh) {
  avNh = &nh;
  mPoseSub = nh.subscribe(ARM_POSE_TOPIC, 1, &armvizwidget::ArmPosCallback, this);
}

void armvizwidget::ArmPosCallback(std_msgs::Float64MultiArrayConstPtr armPos) {
  sideViewArm->setAngles(armPos->data);
  topViewArm->setAngles(armPos->data);
  sideScene->update();
  topViewArm->update();
}


Arm* armvizwidget::createArm(bool isSideView, std::vector<double> angles){
  ArmLink shoulder = {SHOULDER_LEN, SHOULDER_THICK, SHOULDER_JOINT_RAD, Qt::black, Qt::red};
  ArmLink elbow = {ELBOW_LEN, ELBOW_THICK, ELBOW_JOINT_RAD, Qt::black, Qt::red};
  ArmLink wrist = {WRIST_LEN, WRIST_THICK, WRIST_JOINT_RAD, Qt::black, Qt::red};
  Claw claw = {CLAW_LEN, CLAW_WIDTH, CLAW_THICK, Qt::black};
  TurnTable turnTable = {TURNTABLE_RAD, TURNTABLE_OFFSET, Qt::black, Qt::gray};
  RobotFrame robotFrame = {ROBOT_LEN, ROBOT_WIDTH, FRAME_OFFSET, Qt::black, Qt::gray};

  Arm* arm = new Arm(shoulder, elbow, wrist, claw, turnTable, robotFrame, QPointF(0, 0), isSideView, angles);
  return arm;
}