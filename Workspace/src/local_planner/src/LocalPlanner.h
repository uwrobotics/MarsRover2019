//
// Created by tom on 25/07/18.
//

#ifndef PROJECT_LOCALPLANNER_H
#define PROJECT_LOCALPLANNER_H


#include "PlanningAlgoBase.h"
#include "local_planner/LocalPlannerStatus.h"
#include "RoverParams.h"
//#include "occupancy_grid/OccupancyGrid.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/Point.h>
#include <mutex>
#include <thread>

class CLocalPlanner {
public:
    explicit CLocalPlanner(ros::NodeHandle *pNh,
                           const RobotParams_t &robotParams);
  // Main thread
  void Exec();
private:
    // Subscriber callbacks
    void GoalGPSCallback(sensor_msgs::NavSatFixConstPtr goal);
    //void CurGPSCallback(sensor_msgs::NavSatFixConstPtr gps);
    //void OccupancyCallback(occupancy_grid::OccupancyGrid::ConstPtr grid);
    void OdometryCallback(nav_msgs::Odometry::ConstPtr odemetry);
    //void OdometryCallback(sensor_msgs::Imu::ConstPtr odometry);
    void EnableCallback(std_msgs::BoolConstPtr pEnableMsg);
    // publisher thread
    void VelocityPublisher();

    // Main update logic
    void UpdateVelocity();

    // Ros handlers
    ros::NodeHandle *m_pNh;
    ros::Subscriber *m_pOccupancySub;
    ros::Subscriber *m_pGoalGpsSub;
    ros::Subscriber *m_pOdometrySub;
    ros::Subscriber *m_pGpsSub;
    ros::Subscriber *m_pEnableSub;
    ros::Publisher *m_pVelPub;
    ros::Publisher *m_pStatusPub;

    // status
    geometry_msgs::Twist m_curVel;
    double m_curGpsUtmX;
    double m_curGpsUtmY;
    std::string m_curGpsUtmZone;
    double m_goalGpsUtmX;
    double m_goalGpsUtmY;
    std::string m_goalGpsUtmZone;
    double m_orientationToGoal;
    // Parameters
    const RobotParams_t &m_robotParams;

    bool m_bOdomReceived;
    bool m_bGpsReceived;
    bool m_bGoalReceived;
    bool m_bGoalReached;
    bool m_bGoalInRange;
    double m_distToGoal;
    bool m_bEnabled;

    // velocity publishing
    std::mutex m_velMutex;
    geometry_msgs::Twist m_targetVel;
    bool m_bVelocityReady;
    std::thread *m_pVelPubThread;

    // tracking lost obstacles
    //double m_distanceSinceLastRightDanger;
    //double m_distanceSinceLastLeftDanger;
    double m_lastDwaCoordUtmX;
    double m_lastDwaCoordUtmY;

    // distance from goal thresholds
    double m_goalReachedDistThresh;
    double m_goalSearchDistThresh;

    //Planning Algorithm
    CPlanningAlgoBase* m_pPlanningAlgo;

};


#endif //PROJECT_LOCALPLANNER_H
