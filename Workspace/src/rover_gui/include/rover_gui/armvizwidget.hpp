#ifndef ARMVIZWIDGET_HPP
#define ARMVIZWIDGET_HPP

#include <QWidget>

namespace Ui {
class armvizwidget;
}

class armvizwidget : public QWidget
{
    Q_OBJECT

public:
    explicit armvizwidget(QWidget *parent = nullptr);
    ~armvizwidget();

private:
    Ui::armvizwidget *ui;
};

#endif // ARMVIZWIDGET_HPP
