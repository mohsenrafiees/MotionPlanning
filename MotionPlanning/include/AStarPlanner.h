#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include <vector>
#include <unordered_map>
#include <Eigen/Dense>
#include <functional>
#include "BasePlanner.h"
#include "Vector3dHash.h"
#include "OccupancyGrid.h"
#include "PlannerUtils.h"
#include "Node.h"

// A* Planner class definition
class AStarPlanner : public BasePlanner {
public:
    AStarPlanner(const OccupancyGrid &grid, double robot_radius)
        : grid_(grid), robot_radius_(robot_radius) {}

    std::vector<Eigen::Vector3d> getCollisionFreePath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal) override;
private:    
    std::vector<Eigen::Vector3d> reconstructPath(const std::unordered_map<Eigen::Vector3d, Eigen::Vector3d, Vector3dHash>& came_from, const Eigen::Vector3d& current_node) const;
    std::vector<Eigen::Vector3d> getMotionPrimitives(const Eigen::Vector3d& current_node) const;
    double heuristic(const Eigen::Vector3d& neighbor, const Eigen::Vector3d& goal) const;
    std::vector<Eigen::Vector3d> smoothPath(const std::vector<Eigen::Vector3d>& path) const;
    // std::vector<Eigen::Vector3d> smoothPath(const std::vector<Eigen::Vector3d>& path_points) const;

protected:
    // Override the pure virtual method from BasePlanner
    virtual bool isCollisionFree(const Eigen::Vector2d& position) override {
        return !grid_.isCollision(position, robot_radius_);
    }
    const OccupancyGrid &grid_;
    double robot_radius_;  
    std::priority_queue<Node, std::vector<Node>, CompareNode> open_list_;
    std::unordered_map<Eigen::Vector3d, double, Vector3dHash> g_cost_;
};

#endif  // ASTAR_PLANNER_H
