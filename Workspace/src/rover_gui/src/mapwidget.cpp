#include "mapwidget.hpp"
#include "gui.h"
#include "ui_mapwidget.h"
#include <QBrush>
#include <QGraphicsLineItem>
#include <robot_localization/navsat_conversions.h>
/****
 * Helper arrow class
 */

class Arrow : public QGraphicsLineItem {
public:
  Arrow(float arrowSize, Qt::GlobalColor color, QGraphicsItem *parent = 0)
      : QGraphicsLineItem(parent), mSize(arrowSize), mColor(color), mTheta(0) {
    SetTheta(mTheta);
  }

  void SetTheta(float theta) {
    mTheta = theta;
    setLine(0, 0, mSize * cos(mTheta), -mSize * sin(mTheta));
  }

protected:
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
             QWidget *widget = 0) override {

    QPen myPen = pen();
    myPen.setColor(mColor);
    myPen.setWidthF(mSize / 10);
    qreal arrowSize = mSize;
    painter->setPen(myPen);
    painter->setBrush(mColor);

    QLineF centerLine(this->line());

    double angle = std::atan2(-line().dy(), line().dx());

    QPointF arrowP1 =
        line().p2() - QPointF(sin(angle + M_PI / 3) * arrowSize / 5,
                              cos(angle + M_PI / 3) * arrowSize / 5);
    QPointF arrowP2 =
        line().p2() - QPointF(sin(angle + M_PI - M_PI / 3) * arrowSize / 5,
                              cos(angle + M_PI - M_PI / 3) * arrowSize / 5);

    arrowHead.clear();
    arrowHead << line().p2() << arrowP1 << arrowP2;
    painter->drawLine(line());  void BaseAngleCallback(std_msgs::Float64ConstPtr msg);

    painter->drawPolygon(arrowHead);
    if (isSelected()) {
      painter->setPen(QPen(mColor, 1, Qt::DashLine));
      QLineF myLine = line();
      myLine.translate(0, 4.0);
      painter->drawLine(myLine);
      myLine.translate(0, -8.0);
      painter->drawLine(myLine);
    }
  }

private:
  QColor mColor;
  float mSize;
  float mTheta;
  QPolygonF arrowHead;
};

MapWidget::MapWidget(QWidget *parent)
    : QWidget(parent), ui(new Ui::MapWidget),
      mRoverArrowItem(new Arrow(2, Qt::red)), mGoalItem(nullptr),
      mBaseStationArrowItem(nullptr) {
  ui->setupUi(this);

  longitude = 0;
  latitude = 0;
  easting_utm = 0;
  northing_utm = 0;

  mScene = new QGraphicsScene(this);

  ui->mapGraphicsView->setScene(mScene);

  mScene->addItem(mRoverArrowItem);

  ui->mapGraphicsView->show();
  mScale = 5.0;
  ui->mapGraphicsView->scale(mScale, mScale);

  // connect(ui->zoomInButton, SIGNAL(clicked()), this, SLOT(ZoomIn()));
}

MapWidget::~MapWidget() { delete ui; }

bool MapWidget::Init(ros::NodeHandle &nh) {
  mpNh = &nh;
  mPoseSub = nh.subscribe(UTM_POSE_TOPIC, 1, &MapWidget::PoseCallback, this);
  mGoalSub = nh.subscribe("/goal/utm", 1, &MapWidget::GoalCallback, this);
  mBaseAngleSub = nh.subscribe("/basestation/antenna_angle", 1, &MapWidget::BaseAngleCallback, this);
  mBaseCalibratePub = nh.advertise<std_msgs::Bool>("/basestation/calibrated", 1);
}

void MapWidget::PoseCallback(geometry_msgs::Pose2DConstPtr receivedMsg) {

  double rover_utm_north = receivedMsg->y;
  double rover_utm_east = receivedMsg->x;
  double heading = receivedMsg->theta; // radians counter-clockwise from due
                                       // east
  if (!mBaseStationArrowItem) {
    mBaseStationArrowItem = new Arrow(2, Qt::green);
    mBaseStationArrowItem->SetTheta(0);
    mScene->addItem(mBaseStationArrowItem);
    mBaseStationArrowItem->setPos(rover_utm_east, -rover_utm_north);
  }

  mRoverArrowItem->SetTheta(heading);
  mRoverArrowItem->setPos(rover_utm_east, -rover_utm_north);
  ui->mapGraphicsView->setSceneRect(mScene->itemsBoundingRect());
  ui->mapGraphicsView->fitInView(mScene->itemsBoundingRect(),
                                 Qt::KeepAspectRatio);
}

void MapWidget::GoalCallback(geometry_msgs::Pose2DConstPtr receivedMsg) {
  if (!mGoalItem) {
    mGoalItem = mScene->addEllipse(-1, -1, 2, 2, QPen(), QBrush(Qt::blue));
  }

  mGoalItem->setPos(receivedMsg->x, -receivedMsg->y);
}

void MapWidget::BaseAngleCallback(std_msgs::Float64ConstPtr msg) {
  mBaseStationArrowItem->SetTheta(msg->data * M_PI/180.0);
}


void MapWidget::SetLatLon(double lat, double lon) {
  longitude = lon;
  latitude = lat;

  RobotLocalization::NavsatConversions::LLtoUTM(
      latitude, longitude, northing_utm, easting_utm, utm_zone);

  if (!mGoalItem) {
    mGoalItem = mScene->addEllipse(-1, -1, 2, 2, QPen(), QBrush(Qt::blue));
  }

  mGoalItem->setPos(easting_utm, -northing_utm);

  // ROS_INFO(" you input latitude %f", latitude);
  // ROS_INFO(" you input longitude %f", longitude);
  // ROS_INFO(" easting %f northing %f", easting_utm, northing_utm);
}


void MapWidget::ZoomIn() {
  mScale *= 2;
  // ui->mapGraphicsView->scale(2, 2);
}

void MapWidget::ZoomOut() {
  mScale *= 0.5;
  // ui->mapGraphicsView->scale(0.5, 0.5);
}


void MapWidget::on_baseCalibrateButton_pressed() {
  std_msgs::Bool msg;
  msg.data = true;
  mBaseCalibratePub.publish(msg);
}
