// OccupancyGrid.cpp

#include "OccupancyGrid.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <iostream> // Included for debug output

void OccupancyGrid::loadFromImage(const std::string& filename, double resolution_m_input, double inflation_radius_m) {
    cv::Mat image = cv::imread(filename, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        throw std::runtime_error("Failed to load occupancy grid image!");
    }

    // Print original image dimensions for debugging
    std::cout << "Original image dimensions: " << image.cols << " x " << image.rows << std::endl;

    // Enforce 320 x 320 size if dimensions are different
    if (image.cols != 320 || image.rows != 320) {
        std::cout << "Resizing image to 320 x 320." << std::endl;
        cv::resize(image, image, cv::Size(320, 320), 0, 0, cv::INTER_NEAREST);
    }

    // Print resized image dimensions for confirmation
    std::cout << "Adjusted image dimensions: " << image.cols << " x " << image.rows << std::endl;

    // Set resolution_m based on input
    resolution_m = resolution_m_input;

    width = image.cols;
    height = image.rows;
    
    data.resize(width * height);

    // Convert image to occupancy grid
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            uchar pixel = image.at<uchar>(y, x);
            data[y * width + x] = (pixel < 128); // Occupied if pixel is dark
        }
    }

    // Inflate obstacles
    int inflation_radius_cells = static_cast<int>(std::ceil(inflation_radius_m / resolution_m));
    std::vector<bool> inflated_data = data;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (data[y * width + x]) {
                for (int dy = -inflation_radius_cells; dy <= inflation_radius_cells; ++dy) {
                    for (int dx = -inflation_radius_cells; dx <= inflation_radius_cells; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            inflated_data[ny * width + nx] = true;
                        }
                    }
                }
            }
        }
    }

    data = inflated_data;
}

bool OccupancyGrid::isCollision(const Eigen::Vector2d& position, double robot_radius) const {
    int x = static_cast<int>(std::round(position.x() / resolution_m));
    int y = height - static_cast<int>(std::round(position.y() / resolution_m));

    int robot_radius_cells = static_cast<int>(std::ceil(robot_radius / resolution_m));

    for (int dy = -robot_radius_cells; dy <= robot_radius_cells; ++dy) {
        for (int dx = -robot_radius_cells; dx <= robot_radius_cells; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                if (data[ny * width + nx]) {
                    // Debug output
                    // std::cout << "Collision detected at (" << nx << ", " << ny << ")" << std::endl;
                    return true; // Collision detected
                }
            } else {
                // Debug output
                // std::cout << "Position (" << nx << ", " << ny << ") is out of bounds." << std::endl;
                return true; // Out of bounds, treat as collision
            }
        }
    }

    return false; // No collision
}

cv::Mat OccupancyGrid::computeDistanceMap() const{
    // Create an image where obstacles are white (255) and free space is black (0)
    cv::Mat binary_map(height, width, CV_8UC1);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (data[y * width + x]) {
                binary_map.at<uchar>(y, x) = 255; // Obstacle
            } else {
                binary_map.at<uchar>(y, x) = 0;   // Free space
            }
        }
    }

    // Compute the distance transform
    cv::Mat distance_map;
    cv::distanceTransform(~binary_map, distance_map, cv::DIST_L2, 5);

    // Convert distances from pixels to meters
    distance_map = distance_map * resolution_m;

    return distance_map;
}

#include <cmath>  // For sqrt

double OccupancyGrid::getObstacleCost(double x, double y) const {
    // Convert from world coordinates to grid coordinates
    int grid_x = static_cast<int>(x / resolution_m);
    int grid_y = static_cast<int>(y / resolution_m);

    // Check if the coordinates are within grid bounds
    if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height) {
        return 1e6;  // Large cost for out-of-bounds points
    }

    // If the cell itself is occupied, return high cost
    if (data[grid_y * width + grid_x]) {
        return 1e3;  // High cost for occupied cells
    }

    // Calculate cost based on proximity to nearest obstacle
    double min_distance = std::numeric_limits<double>::max();

    // Search nearby cells for obstacles (can be optimized)
    int search_radius = 10;  // Number of cells to check around the point
    for (int dy = -search_radius; dy <= search_radius; ++dy) {
        for (int dx = -search_radius; dx <= search_radius; ++dx) {
            int nx = grid_x + dx;
            int ny = grid_y + dy;

            // Check if the neighbor is within bounds and occupied
            if (nx >= 0 && nx < width && ny >= 0 && ny < height && data[ny * width + nx]) {
                double distance = std::sqrt(dx * dx + dy * dy) * resolution_m;
                min_distance = std::min(min_distance, distance);
            }
        }
    }

    // Define cost based on distance to nearest obstacle
    double obstacle_cost = std::exp(-min_distance);  // Decreasing cost with distance

    return obstacle_cost;
}

#include "OccupancyGrid.h"
#include <queue>
#include <cmath>

double OccupancyGrid::distanceToNearestObstacle(const Eigen::Vector3d& position) const {
    int x = static_cast<int>(position.x() / resolution_m);
    int y = static_cast<int>(position.y() / resolution_m);

    // Boundary check
    if (x < 0 || x >= width || y < 0 || y >= height) {
        return std::numeric_limits<double>::infinity(); // Out of bounds
    }

    // If the starting cell is already an obstacle
    if (data[y * width + x] == 1) {
        return 0.0;
    }

    // BFS to find the nearest obstacle
    std::queue<std::pair<int, int>> q;
    std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));
    q.push({x, y});
    visited[y][x] = true;

    int directions[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    int distance = 0;

    while (!q.empty()) {
        int qsize = q.size();
        for (int i = 0; i < qsize; i++) {
            auto [cx, cy] = q.front();
            q.pop();

            // Check if this cell is an obstacle
            if (data[cy * width + cx] == 1) {
                return distance * resolution_m; // Return distance in meters
            }

            // Explore neighbors
            for (auto& dir : directions) {
                int nx = cx + dir[0];
                int ny = cy + dir[1];

                if (nx >= 0 && nx < width && ny >= 0 && ny < height && !visited[ny][nx]) {
                    visited[ny][nx] = true;
                    q.push({nx, ny});
                }
            }
        }
        distance++;
    }

    return std::numeric_limits<double>::infinity(); // No obstacle found
}

