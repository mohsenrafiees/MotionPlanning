```cpp
#ifndef PRM_PLANNER_H
#define PRM_PLANNER_H

#include "BasePlanner.h"
#include "OccupancyGrid.h"
#include <vector>
#include <unordered_map>
#include <memory>
#include <Eigen/Dense>

struct PRMNode {
    Eigen::Vector3d position;
    std::vector<std::pair<std::shared_ptr<PRMNode>, double>> neighbors;  // neighbor and edge cost

    PRMNode(const Eigen::Vector3d& pos) : position(pos) {}
};

class PRMPlanner : public BasePlanner {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PRMPlanner(const OccupancyGrid& grid, double robot_radius,
               int num_samples = 1000, double connection_radius = 1.0,
               int k_nearest = 10);

    std::vector<Eigen::Vector3d> getCollisionFreePath(
        const Eigen::Vector3d& start, 
        const Eigen::Vector3d& goal) override;

private:
    void buildRoadmap();
    Eigen::Vector3d sampleRandomPoint();
    std::vector<std::shared_ptr<PRMNode>> findNearestNodes(
        const Eigen::Vector3d& point, int k);
    bool isPathCollisionFree(const Eigen::Vector3d& from, const Eigen::Vector3d& to);
    std::vector<Eigen::Vector3d> findPathAstar(
        std::shared_ptr<PRMNode> start_node,
        std::shared_ptr<PRMNode> goal_node);
    void connectNode(std::shared_ptr<PRMNode> node);

    const OccupancyGrid& grid_;
    double robot_radius_;
    int num_samples_;
    double connection_radius_;
    int k_nearest_;
    std::vector<std::shared_ptr<PRMNode>> nodes_;
    std::mt19937 rng_;
    bool roadmap_built_;
};

#endif // PRM_PLANNER_H
```