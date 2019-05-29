#include "PlanningAlgoBase.h"
#include "CostMap.h"
#include <ros/ros.h>


class DWAPlanner : public CPlanningAlgoBase {
public:
   DWAPlanner(const RobotParams_t *robotParams, ros::NodeHandle *pNh);
   geometry_msgs::Twist
        CalculateBestVelocity(const geometry_msgs::Twist &curVel,
                        double headingToGoal, double sqrDistToGoal);

private:
  class DynamicWindowPoint {
  public:
    DynamicWindowPoint(float vel, float angVel, float turnRad)
            : v(vel), w(angVel), rad(turnRad), dist(0), score(0), feasible(false) {}
        float v;
        float w;
        float rad;
        double dist;
        double score;
        bool feasible;
    };
      // The dynamic window grid
    typedef std::vector<DynamicWindowPoint> VelocityRow;
    std::vector<VelocityRow> m_dynamicWindowGrid;
  
    void ConstructDWAGrid();
    void CostMapCallback(cost_map::CostmapConstPtr msg);
    ros::Subscriber m_costMapSub;
    CostMap m_costmap;
    double m_costMapThresh;

    // Parameters
    float m_vIncrement;
    float m_wIncrement;
    float m_lowV;
    float m_highV;
    float m_lowW;
    float m_highW;
    float m_timestep;

    // Status
    float m_curV;
    float m_curW;

};