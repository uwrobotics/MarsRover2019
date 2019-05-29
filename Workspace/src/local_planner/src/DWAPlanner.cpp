#include "DWAPlanner.h"
#include <algorithm>


DWAPlanner::DWAPlanner(const RobotParams_t *robotParams, ros::NodeHandle *pNh)
 : CPlanningAlgoBase(robotParams, pNh), m_costmap(0.7, robotParams), m_vIncrement(0.1), m_wIncrement(0.1),
      m_timestep(robotParams->timestep) {
  m_costMapThresh = 0.7;
  ros::param::get("/local_planner/costmap_thresh", m_costMapThresh);

  m_costMapSub = pNh->subscribe("/costmap", 1, &DWAPlanner::CostMapCallback, this);

  ROS_INFO("Initialized DWA planner with costmap thresh %f", m_costMapThresh);
  m_costmap.SetThreshold(m_costMapThresh);


}

void DWAPlanner::CostMapCallback(cost_map::CostmapConstPtr costmap_msg) {
  m_costmap.SetCostMap(costmap_msg);
}


void DWAPlanner::ConstructDWAGrid() {
    m_lowV = std::max(m_curV - m_pRobotParams->maxLinDecel * m_timestep,
                      m_pRobotParams->minV);
    m_highV = std::min(m_curV + m_pRobotParams->maxLinAccel * m_timestep,
                      m_pRobotParams->maxV);
    m_lowW = std::max(m_curW - m_pRobotParams->maxAngAccel * m_timestep,
                      -m_pRobotParams->maxW);
    m_highW = std::min(m_curW + m_pRobotParams->maxAngAccel * m_timestep,
                      m_pRobotParams->maxW);

    // construct the dynamic window
    m_dynamicWindowGrid.resize(std::round((m_highV - m_lowV) / m_vIncrement) + 1);
    int row = 0;
    for (auto &velocityRow : m_dynamicWindowGrid) {
      double velocity = m_lowV + row * m_vIncrement;
      // if this row is for approximately zero velocity, populate using radial
      // velocities
      if (std::round(velocity * 1000) == 0) {
        velocity = 0;
        unsigned long rowSize = std::round((m_highW - m_lowW) / m_wIncrement) + 1;
        velocityRow.reserve(rowSize);
        for (int col = 0; col < rowSize; col++) {
          velocityRow.emplace_back(velocity, m_lowW + col * m_wIncrement, 0);
        }

      } else {
        // if the speed is non-zero, then instead populate based on turning radii
        // for a better selection of trajectories
        // note: rad = v/w
        double maxRad = 10;
        double radIncrement = 0.5;

        unsigned long rowSize = 2 * std::round(maxRad / radIncrement) + 1;
        velocityRow.reserve(rowSize);

        double curRad = -radIncrement;
        for (double curRad = -radIncrement; curRad > -maxRad;
             curRad -= radIncrement) {
          velocityRow.emplace_back(velocity, velocity / curRad, curRad);
        }

        // add a straight trajectory
        velocityRow.emplace_back(velocity, 0, 0);

        for (double curRad = radIncrement; curRad < maxRad;
             curRad += radIncrement) {
          velocityRow.emplace_back(velocity, velocity / curRad, curRad);
        }
      }
    row++;
  }
}


geometry_msgs::Twist DWAPlanner::CalculateBestVelocity(const geometry_msgs::Twist &curVel,
                        double headingToGoal, double sqrDistToGoal) {
  geometry_msgs::Twist chosenVel;

  if (m_costmap.IsReady()) {
    // Construct DWA
    m_curV = curVel.linear.x;
    m_curW = curVel.angular.z;
    // determine the bounds on velocities




  }

  return chosenVel;
}
