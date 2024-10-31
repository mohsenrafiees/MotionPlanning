#ifndef PLANNER_UTILS_H
#define PLANNER_UTILS_H

// Forward declaration of AStarPlanner class
class AStarPlanner;

#include <vector>
#include <Eigen/Dense>



std::vector<Eigen::Vector3d> TestPathExists(AStarPlanner& planner, const Eigen::Vector3d& start, const Eigen::Vector3d& goal);
std::vector<Eigen::Vector3d> TestNoPath(AStarPlanner& planner, const Eigen::Vector3d& start, const Eigen::Vector3d& goal);


#endif
