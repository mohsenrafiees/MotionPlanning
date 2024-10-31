#ifndef THETASTARPLANNER_H
#define THETASTARPLANNER_H

#include "AStarPlanner.h"
#include "Node.h"
#include <vector>
#include <Eigen/Dense>
#include <unordered_set>


class ThetaStarPlanner : public AStarPlanner{
public:
    ThetaStarPlanner(const OccupancyGrid& grid, double robot_radius);

    std::vector<Eigen::Vector3d> getCollisionFreePath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal) override;

private:
    bool lineOfSight(const Eigen::Vector2d& start, const Eigen::Vector2d& end);
    std::vector<Eigen::Vector3d> reconstructPath(const Node& goal_node);
    void expandNeighbors(Node& current, const Eigen::Vector3d& goal);
    std::vector<Eigen::Vector3d> prunePath(const std::vector<Eigen::Vector3d>& path);
    std::vector<Eigen::Vector3d> smoothPathWithLineOfSight(const std::vector<Eigen::Vector3d>& path);
};

#endif  // THETASTARPLANNER_H
