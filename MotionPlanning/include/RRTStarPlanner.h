// RRTStarPlanner.h
#ifndef RRTSTAR_PLANNER_H
#define RRTSTAR_PLANNER_H

#include "BasePlanner.h"
#include "OccupancyGrid.h"
#include <vector>
#include <memory>
#include <random>
#include <Eigen/Dense>
#include <Eigen/StdVector>

struct RRTNode {
    Eigen::Vector3d position;
    double cost;
    std::shared_ptr<RRTNode> parent;
    std::vector<std::shared_ptr<RRTNode>> children;

    RRTNode(const Eigen::Vector3d& pos, double cost = 0.0, 
            std::shared_ptr<RRTNode> parent = nullptr)
        : position(pos), cost(cost), parent(parent) {}
};

class RRTStarPlanner : public BasePlanner {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RRTStarPlanner(const OccupancyGrid& grid, double robot_radius);

    std::vector<Eigen::Vector3d> getCollisionFreePath(
        const Eigen::Vector3d& start, const Eigen::Vector3d& goal) override;
protected:
    virtual bool isCollisionFree(const Eigen::Vector2d& position) override {
        return !grid_.isCollision(position, robot_radius_);
    }

private:
    const OccupancyGrid& grid_;
    double robot_radius_;
    int max_iterations_;
    double step_size_;
    double goal_threshold_;
    double rewire_radius_;
    std::mt19937 random_generator_;
    std::vector<std::shared_ptr<RRTNode>> nodes_;
    
    // World bounds in meters
    double world_min_x_;
    double world_min_y_;
    double world_max_x_;
    double world_max_y_;

    bool isPointInBounds(const Eigen::Vector2d& point) const;
    Eigen::Vector3d sampleRandomPoint();
    std::shared_ptr<RRTNode> findNearest(const Eigen::Vector3d& point);
    std::vector<std::shared_ptr<RRTNode>> findNeighbors(const Eigen::Vector3d& point);
    Eigen::Vector3d steer(const Eigen::Vector3d& from, const Eigen::Vector3d& to);
    bool isCollisionFree(const Eigen::Vector2d& from, const Eigen::Vector2d& to);
    double calculateCost(std::shared_ptr<RRTNode> from, std::shared_ptr<RRTNode> to);
    std::vector<Eigen::Vector3d> reconstructPath(std::shared_ptr<RRTNode> goal_node);
    void rewireNodes(std::shared_ptr<RRTNode> new_node, 
                    const std::vector<std::shared_ptr<RRTNode>>& neighbors);
    void updateChildrenCosts(std::shared_ptr<RRTNode> node, double cost_change);
};

#endif // RRTSTAR_PLANNER_H

