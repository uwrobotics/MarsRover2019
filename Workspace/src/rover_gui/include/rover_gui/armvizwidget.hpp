#ifndef ARMVIZWIDGET_HPP
#define ARMVIZWIDGET_HPP

// remember to change to correct topic name
#define ARM_POSE_TOPIC "/localization/pose_arm"

#include <QWidget>

namespace Ui {
class armvizwidget;
}

class armvizwidget : public QWidget{
	Q_OBJECT

	public:
	    explicit armvizwidget(QWidget *parent = nullptr);
	    ~armvizwidget();

	private:
	    Ui::armvizwidget *ui;
};

#endif // ARMVIZWIDGET_HPP
