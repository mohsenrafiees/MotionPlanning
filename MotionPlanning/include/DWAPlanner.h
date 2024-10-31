
#ifndef DWA_PLANNER_HPP
#define DWA_PLANNER_HPP

#include "BasePlanner.h"
#include <vector>

class DWAPlanner : public BasePlanner {
public:
    std::vector<Eigen::Vector3d> getCollisionFreePath(const Eigen::Vector3d& start, 
                                                      const Eigen::Vector3d& goal) override;
    
private:
    // Helper methods for DWA algorithm
    std::vector<Eigen::Vector3d> generateTrajectory(const Eigen::Vector3d& current_state);
    Eigen::Vector3d dynamicWindowApproach(const Eigen::Vector3d& current_state, const Eigen::Vector3d& goal);
};

#endif // DWA_PLANNER_HPP
