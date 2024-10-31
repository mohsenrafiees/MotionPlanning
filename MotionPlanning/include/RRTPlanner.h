
#ifndef RRT_PLANNER_HPP
#define RRT_PLANNER_HPP

#include "BasePlanner.h"
#include <random>

class RRTPlanner : public BasePlanner {
public:
    std::vector<Eigen::Vector3d> getCollisionFreePath(const Eigen::Vector3d& start, 
                                                      const Eigen::Vector3d& goal) override;
protected:
    // Override the pure virtual method from BasePlanner
    virtual bool isCollisionFree(const Eigen::Vector2d& position) override {
        return !grid_.isCollision(position, robot_radius_);
    }    

private:
    // Helper methods for RRT algorithm
    Eigen::Vector3d getRandomNode();
    bool isValidConnection(const Eigen::Vector3d& node1, const Eigen::Vector3d& node2);
};

#endif // RRT_PLANNER_HPP
