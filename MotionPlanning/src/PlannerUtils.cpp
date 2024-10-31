#include "AStarPlanner.h"
#include <iostream>
#include <vector>
#include <Eigen/Dense>

std::vector<Eigen::Vector3d> TestPathExists(AStarPlanner& planner, const Eigen::Vector3d& start, const Eigen::Vector3d& goal) {
    std::vector<Eigen::Vector3d> path = planner.getCollisionFreePath(start, goal);
    if (path.size() > 0) {
        std::cout << "Path exists between start and goal!" << std::endl;
    } else {
        std::cout << "No path found between start and goal." << std::endl;
    }
    return path;
}

std::vector<Eigen::Vector3d> TestNoPath(AStarPlanner& planner, const Eigen::Vector3d& start, const Eigen::Vector3d& goal) {
    std::vector<Eigen::Vector3d> path = planner.getCollisionFreePath(start, goal);
    if (path.size() == 0) {
        std::cout << "As expected, no path found due to obstacles." << std::endl;
    } else {
        std::cout << "Unexpectedly found a path!" << std::endl;
    }
    return path;
}
