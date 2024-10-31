// SBPLPlanner.h

#ifndef SBPLPLANNER_H
#define SBPLPLANNER_H

#include "BasePlanner.h"
#include "OccupancyGrid.h"
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <Eigen/Dense>
#include <Eigen/StdVector> // Include for aligned allocator
#include <functional>
#include <memory>

// Define a struct for discretized positions
struct GridState {
    int x;
    int y;
    int theta; // Discrete orientation if needed

    bool operator==(const GridState& other) const;
};

// Hash function for GridState
struct GridStateHash {
    std::size_t operator()(const GridState& state) const;
};

// Node in the search tree
struct SBPLNode {
    GridState state;
    double g_cost; // Cost from start to this node
    double f_cost; // Estimated total cost (g_cost + h_cost)
    std::shared_ptr<SBPLNode> parent;

    SBPLNode(const GridState& state, double g_cost, double f_cost, std::shared_ptr<SBPLNode> parent = nullptr);
};

// Comparator for priority queue
struct CompareSBPLNode {
    bool operator()(const std::shared_ptr<SBPLNode>& a, const std::shared_ptr<SBPLNode>& b) const;
};

class SBPLPlanner : public BasePlanner {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SBPLPlanner(const OccupancyGrid& grid, double robot_radius);
    std::vector<Eigen::Vector3d> getCollisionFreePath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal);

protected:
    // Override the pure virtual method from BasePlanner
    virtual bool isCollisionFree(const Eigen::Vector2d& position) override {
        return !grid_.isCollision(position, robot_radius_);
    }

private:
    void initializeMotionPrimitives();
    std::vector<Eigen::Vector3d> reconstructPath(std::shared_ptr<SBPLNode> goal_node);
    double heuristic(const GridState& state, const GridState& goal_state) const;

    const OccupancyGrid& grid_;
    double robot_radius_;
    double goal_threshold_;
    double cost_threshold_;
    double resolution_;

    // Use aligned allocator for Eigen types
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> motion_primitives_;

    // Node map to keep nodes alive
    std::unordered_map<GridState, std::shared_ptr<SBPLNode>, GridStateHash> node_map_;
};

#endif // SBPLPLANNER_H
