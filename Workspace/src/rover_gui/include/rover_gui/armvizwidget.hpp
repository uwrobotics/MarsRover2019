#ifndef ARMVIZWIDGET_HPP
#define ARMVIZWIDGET_HPP

#include <QWidget>
#include <QString>
#include <QtCore>
#include <QBrush>
#include <QPointF>
#include<QGraphicsScene>
#include <vector>
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#endif

namespace Ui {
class armvizwidget;
}

class armvizwidget : public QWidget{
	Q_OBJECT

	public:
	    explicit armvizwidget(QWidget *parent = nullptr);
	    ~armvizwidget();

	    bool Init(ros::NodeHandle &nh);

	private:
		// expects an ID array of six angles in radians
		void ArmPosCallback(std_msgs::Float64MultiArrayConstPtr armPos);

	    Ui::armvizwidget *ui;
	    ros::NodeHandle *avNh;
	    QGraphicsScene *topScene;
	    QGraphicsScene *sideScene;
	    QPen *mPen;
	    QPointF startPos;

  		ros::Subscriber mPoseSub;

  		bool isTopDownView;

  		// index for input angles from subscribed message
  		const double TURNTABLE_YAW=0;
  		const double SHOULDER_PITCH=1;
  		const double ELBOW_PITCH=2;
  		const double WRIST_PITCH=3;
  		const double WRIST_ROLL=4;
  		const double CLAW_CLOSURE_ANGLE=5;

  		//dimensions
  		const double ROBOT_LEN=0, ROBOT_WIDTH=0;
  		const double TURNTABLE_RAD=0;
  		const double SHOULDER_LEN=25, ELBOW_LEN=25, WRIST_LEN=25, CLAW_LEN=15;
  		const double SHOULDER_THICK=2, ELBOW_THICK=2, WRIST_THICK=2;

  		struct ArmLink{
  			double length;
  			double thickness;
  			QColor mColor;
  		};

  		ArmLink *shoulder, *elbow, *wrist;

  		QPointF DrawLink(ArmLink *armLink, QPen *mPen, QPointF startPos, double pitch, QGraphicsScene *mScene);
  		void DrawArmSideView(std::vector<double> angles);
  		void DrawArmTopView(std::vector<double> angles);
};

#endif // ARMVIZWIDGET_HPP
