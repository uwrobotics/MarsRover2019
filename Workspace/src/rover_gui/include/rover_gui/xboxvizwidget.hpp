#ifndef XBOXVIZWIDGET_HPP
#define XBOXVIZWIDGET_HPP

#include <QWidget>

namespace Ui {
class xboxvizwidget;
}

class xboxvizwidget : public QWidget {
  Q_OBJECT

public:
  explicit xboxvizwidget(QWidget *parent = nullptr);
  ~xboxvizwidget();

private:
  Ui::xboxvizwidget *ui;
};

#endif // XBOXVIZWIDGET_HPP
