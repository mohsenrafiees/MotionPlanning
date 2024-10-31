#include "AStarPlanner.h"
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <Eigen/Dense>
#include <opencv2/imgproc.hpp> 
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <sstream>
#include <chrono>
#include <iomanip>

// Helper function to discretize states
Eigen::Vector3d discretizeState(const Eigen::Vector3d& state, double pos_resolution, double angle_resolution) {
    Eigen::Vector3d discretized;
    discretized.x() = std::round(state.x() / pos_resolution) * pos_resolution;
    discretized.y() = std::round(state.y() / pos_resolution) * pos_resolution;
    discretized.z() = std::round(state.z() / angle_resolution) * angle_resolution;
    return discretized;
}

// Helper function to create a string key for visited states
std::string stateToKey(const Eigen::Vector3d& state) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3) 
       << state.x() << "_" << state.y() << "_" << state.z();
    return ss.str();
}

std::vector<Eigen::Vector3d> AStarPlanner::getCollisionFreePath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal) {
    std::cout << "Starting A* from " << start.transpose() << " to " << goal.transpose() << std::endl;
    
    auto start_time = std::chrono::steady_clock::now();
    const double timeout_seconds = 15.0;

    if (grid_.isCollision(start.head<2>(), robot_radius_)) {
        std::cerr << "Start point is in collision: " << start.transpose() << std::endl;
        return {};
    }

    // Check if direct path is possible
    if (!grid_.isCollision(goal.head<2>(), robot_radius_)) {
        Eigen::Vector2d direction = goal.head<2>() - start.head<2>();
        double distance = direction.norm();
        direction.normalize();
        bool direct_path_possible = true;
        
        // Check collision along direct path
        for (double d = 0; d <= distance; d += grid_.resolution_m) {
            Eigen::Vector2d point = start.head<2>() + direction * d;
            if (grid_.isCollision(point, robot_radius_)) {
                direct_path_possible = false;
                break;
            }
        }
        
        if (direct_path_possible) {
            std::cout << "Direct path found!" << std::endl;
            return {start, goal};
        }
    }

    // State discretization parameters
    const double pos_resolution = grid_.resolution_m;
    const double angle_resolution = M_PI / 8;  // 45 degrees

    open_list_ = std::priority_queue<Node, std::vector<Node>, CompareNode>();
    g_cost_.clear();
    std::unordered_map<Eigen::Vector3d, Eigen::Vector3d, Vector3dHash> came_from;
    
    // Use strings as keys for visited states instead of Eigen::Vector3d
    std::unordered_set<std::string> visited;
    
    // Use discretized states
    Eigen::Vector3d discrete_start = discretizeState(start, pos_resolution, angle_resolution);
    Eigen::Vector3d discrete_goal = discretizeState(goal, pos_resolution, angle_resolution);
    
    open_list_.emplace(discrete_start, 0.0);
    g_cost_[discrete_start] = 0.0;

    while (!open_list_.empty()) {
        auto current_time = std::chrono::steady_clock::now();
        double elapsed_seconds = std::chrono::duration<double>(current_time - start_time).count();
        
        if (elapsed_seconds > timeout_seconds) {
            std::cerr << "A* search timeout after " << elapsed_seconds << " seconds!" << std::endl;
            return {};
        }

        Node current = open_list_.top();
        open_list_.pop();

        // Convert state to string key for visited set
        std::string current_key = stateToKey(current.position);
        if (visited.count(current_key)) continue;
        visited.insert(current_key);

        double distance_to_goal = (current.position.head<2>() - discrete_goal.head<2>()).norm();
        if (distance_to_goal < pos_resolution) {
            std::cout << "Goal reached in " << elapsed_seconds << " seconds!" << std::endl;
            return reconstructPath(came_from, current.position);
        }

        for (const auto& neighbor : getMotionPrimitives(current.position)) {
            Eigen::Vector3d discrete_neighbor = discretizeState(neighbor, pos_resolution, angle_resolution);
            std::string neighbor_key = stateToKey(discrete_neighbor);
            
            if (visited.count(neighbor_key) || 
                grid_.isCollision(discrete_neighbor.head<2>(), robot_radius_)) {
                continue;
            }

            double tentative_g_cost = g_cost_[current.position] + 
                                    (discrete_neighbor.head<2>() - current.position.head<2>()).norm();

            if (!g_cost_.count(discrete_neighbor) || tentative_g_cost < g_cost_[discrete_neighbor]) {
                g_cost_[discrete_neighbor] = tentative_g_cost;
                double h_cost = heuristic(discrete_neighbor, discrete_goal);
                double f_cost = tentative_g_cost + h_cost;

                // Only add to open list if heuristic estimate is reasonable
                if (h_cost < 100 * distance_to_goal) {  // Prevent exploring unlikely directions
                    open_list_.emplace(discrete_neighbor, f_cost);
                    came_from[discrete_neighbor] = current.position;
                }
            }
        }
    }

    std::cout << "No path found!" << std::endl;
    return {};
}

// Optimized motion primitives
std::vector<Eigen::Vector3d> AStarPlanner::getMotionPrimitives(const Eigen::Vector3d& current_node) const {
    std::vector<Eigen::Vector3d> neighbors;
    neighbors.reserve(5);  // Pre-allocate for efficiency

    double step_size = grid_.resolution_m;  // Use grid resolution as step size
    double max_steering_angle = M_PI / 4;  // Reduced from PI/2 for more realistic motion

    // Reduced number of steering angles for efficiency
    std::vector<double> steering_angles = {-max_steering_angle, -max_steering_angle/2, 0, max_steering_angle/2, max_steering_angle};

    for (double steering_angle : steering_angles) {
        double new_theta = current_node.z() + steering_angle;
        new_theta = std::fmod(new_theta + 2 * M_PI, 2 * M_PI);  // More efficient normalization

        Eigen::Vector3d neighbor;
        neighbor.x() = current_node.x() + step_size * cos(new_theta);
        neighbor.y() = current_node.y() + step_size * sin(new_theta);
        neighbor.z() = new_theta;

        neighbors.push_back(neighbor);
    }

    return neighbors;
}

// Improved heuristic function
double AStarPlanner::heuristic(const Eigen::Vector3d& node, const Eigen::Vector3d& goal) const {
    double distance = (node.head<2>() - goal.head<2>()).norm();
    
    // More significant heading penalty
    double heading_difference = std::fabs(node.z() - goal.z());
    if (heading_difference > M_PI) {
        heading_difference = 2 * M_PI - heading_difference;
    }
    
    // Check if heading is pointing towards goal
    Eigen::Vector2d to_goal = (goal.head<2>() - node.head<2>()).normalized();
    double current_heading = node.z();
    double angle_to_goal = std::atan2(to_goal.y(), to_goal.x());
    double heading_alignment = std::fabs(current_heading - angle_to_goal);
    if (heading_alignment > M_PI) {
        heading_alignment = 2 * M_PI - heading_alignment;
    }

    // Combined heuristic that considers both distance and heading alignment
    return distance + 0.5 * heading_difference + heading_alignment;
}

std::vector<Eigen::Vector3d> AStarPlanner::reconstructPath(
    const std::unordered_map<Eigen::Vector3d, Eigen::Vector3d, Vector3dHash>& came_from,
    const Eigen::Vector3d& current_node) const {
    
    std::vector<Eigen::Vector3d> path;
    Eigen::Vector3d current = current_node;
    path.push_back(current);  // Add the goal node

    // Trace back the path
    while (came_from.find(current) != came_from.end()) {
        current = came_from.at(current);
        path.push_back(current);
    }

    // Reverse the path to get start-to-goal ordering
    std::reverse(path.begin(), path.end());

    // Post-process the path to make it smoother
    return smoothPath(path);
}

// Implement a basic path smoothing function if not already defined
std::vector<Eigen::Vector3d> AStarPlanner::smoothPath(const std::vector<Eigen::Vector3d>& path) const {
    if (path.size() <= 2) return path;

    std::vector<Eigen::Vector3d> smoothed_path;
    smoothed_path.push_back(path.front());

    // Simple path smoothing: try to connect points directly if possible
    size_t current_idx = 0;
    while (current_idx < path.size() - 1) {
        size_t next_valid_idx = current_idx + 1;
        
        // Look ahead to find the furthest point we can connect to directly
        for (size_t i = current_idx + 2; i < path.size(); i++) {
            bool can_connect = true;
            Eigen::Vector2d start = path[current_idx].head<2>();
            Eigen::Vector2d end = path[i].head<2>();
            
            // Check if the direct path is collision-free
            Eigen::Vector2d direction = end - start;
            double distance = direction.norm();
            direction.normalize();
            
            // Sample points along the line
            for (double d = 0; d <= distance; d += grid_.resolution_m) {
                Eigen::Vector2d point = start + direction * d;
                if (grid_.isCollision(point, robot_radius_)) {
                    can_connect = false;
                    break;
                }
            }
            
            if (can_connect) {
                next_valid_idx = i;
            } else {
                break;
            }
        }
        
        // Add the furthest valid point we found
        smoothed_path.push_back(path[next_valid_idx]);
        current_idx = next_valid_idx;
    }

    // Interpolate orientations for smoother heading changes
    for (size_t i = 0; i < smoothed_path.size() - 1; i++) {
        Eigen::Vector2d direction = smoothed_path[i + 1].head<2>() - smoothed_path[i].head<2>();
        double target_heading = std::atan2(direction.y(), direction.x());
        smoothed_path[i].z() = target_heading;
    }
    
    // Set final orientation to match goal
    if (!smoothed_path.empty()) {
        smoothed_path.back().z() = path.back().z();
    }

    return smoothed_path;
}