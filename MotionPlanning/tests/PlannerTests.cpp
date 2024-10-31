
#include "AStarPlanner.h"
#include "OccupancyGrid.h"
#include "Visualizer.h"
#include "TrajectoryPlanner.h"
#include <iostream>
#include <cassert>
#include <opencv2/opencv.h>

/**
 * Updated unit tests that include trajectory planning and visualization of the generated trajectories.
 * The grid is loaded from the provided 'grid-example.png' file.
 */

// Helper function to load the occupancy grid from an image file
void loadGridFromImage(OccupancyGrid& grid, const std::string& image_file) {
    cv::Mat image = cv::imread(image_file, cv::IMREAD_GRAYSCALE);
    assert(!image.empty() && "Failed to load grid image");

    for (int x = 0; x < image.cols; ++x) {
        for (int y = 0; y < image.rows; ++y) {
            bool is_occupied = (image.at<uchar>(y, x) < 128);  // Consider darker pixels as obstacles
            grid.setOccupancy(x, y, is_occupied);
        }
    }
}

// Test case 1: Basic pathfinding with trajectory generation
void testBasicPathfindingWithTrajectory(Visualizer& visualizer, TrajectoryPlanner& trajectory_planner) {
    OccupancyGrid grid(320, 320, 0.05);
    loadGridFromImage(grid, "grid-example.png");  // Load grid from image file

    AStarPlanner planner;
    planner.setMap(grid);

    Eigen::Vector3d start(1.0, 1.0, 0.0);
    Eigen::Vector3d goal(5.0, 5.0, 0.0);

    std::vector<Eigen::Vector3d> path = planner.getCollisionFreePath(start, goal);

    // The path should not be empty
    assert(!path.empty() && "Basic pathfinding test failed: No path was found.");

    // Generate trajectory based on the path
    std::vector<Eigen::VectorXd> trajectory = trajectory_planner.generateTrajectory(path);

    // Visualize the path and velocity profiles
    visualizer.plotPath(path);

    // Extract velocity profiles from trajectory
    std::vector<double> time, linear_velocities, angular_velocities;
    double t = 0;
    for (const auto& state : trajectory) {
        time.push_back(t);
        linear_velocities.push_back(state(3));  // v
        angular_velocities.push_back(state(4)); // omega
        t += 0.1;  // Assuming constant time step
    }

    visualizer.plotVelocities(time, linear_velocities, angular_velocities);

    std::cout << "Test 1: Basic pathfinding with trajectory generation passed." << std::endl;
}

int main() {
    Visualizer visualizer;
    TrajectoryPlanner trajectory_planner(1.0, 0.5, 0.5, 0.25);  // Max velocities and accelerations

    // Run the test case with trajectory generation
    testBasicPathfindingWithTrajectory(visualizer, trajectory_planner);

    std::cout << "All tests passed!" << std::endl;
    return 0;
}
