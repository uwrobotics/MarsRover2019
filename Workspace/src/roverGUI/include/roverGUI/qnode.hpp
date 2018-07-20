/**
 * @file /include/roverGUI/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef roverGUI_QNODE_HPP_
#define roverGUI_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <sensor_msgs/NavSatFix.h>
#include "gui.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace roverGUI {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
        QNode(int argc, char** argv );
        virtual ~QNode();
        void init();
        bool init(const std::string &master_url, const std::string &host_url);
        void run();


        /*********************
        ** Logging
        **********************/
        enum LogLevel {
                 Debug,
                 Info,
                 Warn,
                 Error,
                 Fatal
         };

        QStringListModel* loggingModel() { return &logging_model; }
        void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
        void loggingUpdated();
        void rosShutdown();

private:
        int init_argc;
        char** init_argv;
        ros::Publisher chatter_publisher;
    QStringListModel logging_model;
};

}  // namespace roverGUI

#endif /* roverGUI_QNODE_HPP_ */
