
#include "AStarPlanner.h"
#include <Eigen/Dense>
#include <iostream>

/**
 * Predefined set of start and goal points for testing the path planners.
 * Users can choose an example number to run the corresponding start and goal configuration.
 */
void runExample(int exampleNumber, AStarPlanner& planner) {
    Eigen::Vector3d start, goal;

    switch (exampleNumber) {
        case 1:
            start = Eigen::Vector3d(1.0, 1.0, 0.0);
            goal = Eigen::Vector3d(5.0, 5.0, 0.0);
            break;
        case 2:
            start = Eigen::Vector3d(2.0, 2.0, 0.0);
            goal = Eigen::Vector3d(6.0, 6.0, M_PI_4);
            break;
        case 3:
            start = Eigen::Vector3d(3.0, 1.0, 0.0);
            goal = Eigen::Vector3d(10.0, 10.0, M_PI_2);
            break;
        case 4:
            start = Eigen::Vector3d(0.0, 0.0, M_PI_2);
            goal = Eigen::Vector3d(12.0, 8.0, 0.0);
            break;
        case 5:
            start = Eigen::Vector3d(1.0, 3.0, M_PI_4);
            goal = Eigen::Vector3d(15.0, 15.0, M_PI);
            break;
        case 6:
            start = Eigen::Vector3d(4.0, 4.0, 0.0);
            goal = Eigen::Vector3d(20.0, 5.0, M_PI_2);
            break;
        case 7:
            start = Eigen::Vector3d(2.0, 4.0, M_PI);
            goal = Eigen::Vector3d(7.0, 7.0, 0.0);
            break;
        case 8:
            start = Eigen::Vector3d(5.0, 5.0, M_PI_2);
            goal = Eigen::Vector3d(18.0, 3.0, M_PI);
            break;
        case 9:
            start = Eigen::Vector3d(8.0, 2.0, 0.0);
            goal = Eigen::Vector3d(25.0, 25.0, M_PI_2);
            break;
        case 10:
            start = Eigen::Vector3d(3.0, 5.0, M_PI_4);
            goal = Eigen::Vector3d(30.0, 10.0, 0.0);
            break;
        default:
            std::cout << "Invalid example number!" << std::endl;
            return;
    }

    // Execute the planner for the selected example
    std::vector<Eigen::Vector3d> path = planner.getCollisionFreePath(start, goal);

    // Print the path
    std::cout << "Planned path for example " << exampleNumber << ":
";
    for (const auto& point : path) {
        std::cout << "x: " << point.x() << ", y: " << point.y() << ", theta: " << point.z() << std::endl;
    }
}
