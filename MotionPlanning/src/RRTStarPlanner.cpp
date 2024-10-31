#include "RRTStarPlanner.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <chrono>

RRTStarPlanner::RRTStarPlanner(const OccupancyGrid& grid, double robot_radius)
    : grid_(grid), 
      robot_radius_(robot_radius), 
      max_iterations_(5000),  // Reduced max iterations since we'll be more efficient
      step_size_(0.2),       // Smaller step size for better precision
      goal_threshold_(0.2),  // Smaller threshold for better accuracy
      random_generator_(std::random_device{}()) {
    
    world_min_x_ = 0.0;
    world_min_y_ = 0.0;
    world_max_x_ = grid.width * grid.resolution_m;
    world_max_y_ = grid.height * grid.resolution_m;

    // Dynamic rewire radius based on world size and expected node density
    double world_area = (world_max_x_ - world_min_x_) * (world_max_y_ - world_min_y_);
    rewire_radius_ = std::sqrt(world_area / (M_PI * max_iterations_)) * 4.0;
}

std::vector<Eigen::Vector3d> RRTStarPlanner::getCollisionFreePath(
    const Eigen::Vector3d& start, const Eigen::Vector3d& goal) {
    
    std::cout << "Starting RRT* from " << start.transpose() << " to " << goal.transpose() << std::endl;
    
    // Check direct path first
    if (isCollisionFree(start.head<2>(), goal.head<2>())) {
        std::cout << "Direct path found!" << std::endl;
        return {start, goal};
    }

    // Check if vertical or horizontal path is possible with small offset
    Eigen::Vector2d direction = goal.head<2>() - start.head<2>();
    if (std::abs(direction.x()) < 0.1 || std::abs(direction.y()) < 0.1) {
        // Try slightly offset paths
        const double offset = robot_radius_ * 2;
        std::vector<double> offsets = {offset, -offset, offset/2, -offset/2};
        
        for (double d : offsets) {
            Eigen::Vector2d intermediate;
            if (std::abs(direction.x()) < 0.1) {  // Vertical path
                intermediate = start.head<2>() + Eigen::Vector2d(d, direction.y()/2);
            } else {  // Horizontal path
                intermediate = start.head<2>() + Eigen::Vector2d(direction.x()/2, d);
            }
            
            if (isCollisionFree(start.head<2>(), intermediate) && 
                isCollisionFree(intermediate, goal.head<2>())) {
                return {start, 
                        Eigen::Vector3d(intermediate.x(), intermediate.y(), start.z()),
                        goal};
            }
        }
    }

    nodes_.clear();
    auto root = std::make_shared<RRTNode>(start);
    nodes_.push_back(root);

    const auto start_time = std::chrono::steady_clock::now();
    const double max_time = 15.0; // seconds
    
    int goal_reach_attempts = 0;
    std::shared_ptr<RRTNode> best_goal_node = nullptr;
    double best_goal_dist = std::numeric_limits<double>::max();

    for (int i = 0; i < max_iterations_; ++i) {
        // Check timeout
        auto current_time = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(current_time - start_time).count() > max_time) {
            std::cout << "Timeout reached!" << std::endl;
            break;
        }

        // Adaptive goal sampling
        double goal_bias = 0.2 + (0.4 * i / max_iterations_); // Increases from 20% to 60%
        
        Eigen::Vector3d random_point;
        if (std::uniform_real_distribution<>(0, 1)(random_generator_) < goal_bias) {
            // Sample near goal with some noise to escape local minima
            random_point = goal;
            if (goal_reach_attempts > 10) {  // Add noise if having trouble reaching goal
                random_point.head<2>() += Eigen::Vector2d(
                    std::normal_distribution<>(0, robot_radius_)(random_generator_),
                    std::normal_distribution<>(0, robot_radius_)(random_generator_)
                );
            }
        } else {
            random_point = sampleRandomPoint();
            // Bias towards axis-aligned movements
            if (std::uniform_real_distribution<>(0, 1)(random_generator_) < 0.3) {
                random_point.x() = std::uniform_real_distribution<>(0, 1)(random_generator_) < 0.5 ? 
                                 start.x() : goal.x();
            }
        }

        auto nearest = findNearest(random_point);
        if (!nearest) continue;

        Eigen::Vector3d new_point = steer(nearest->position, random_point);
        
        if (!isPointInBounds(new_point.head<2>()) || 
            !isCollisionFree(nearest->position.head<2>(), new_point.head<2>())) {
            continue;
        }

        double new_cost = nearest->cost + calculateCost(nearest, 
            std::make_shared<RRTNode>(new_point));
        auto new_node = std::make_shared<RRTNode>(new_point, new_cost, nearest);
        
        // Find better parent
        auto neighbors = findNeighbors(new_point);
        double min_cost = new_cost;
        std::shared_ptr<RRTNode> best_parent = nearest;
        
        for (const auto& neighbor : neighbors) {
            double potential_cost = neighbor->cost + calculateCost(neighbor, new_node);
            if (potential_cost < min_cost && 
                isCollisionFree(neighbor->position.head<2>(), new_point.head<2>())) {
                min_cost = potential_cost;
                best_parent = neighbor;
            }
        }

        new_node->parent = best_parent;
        new_node->cost = min_cost;
        best_parent->children.push_back(new_node);
        nodes_.push_back(new_node);

        rewireNodes(new_node, neighbors);

        // Check goal
        double dist_to_goal = (new_point.head<2>() - goal.head<2>()).norm();
        if (dist_to_goal < best_goal_dist) {
            best_goal_dist = dist_to_goal;
            best_goal_node = new_node;
        }
        
        if (dist_to_goal < goal_threshold_) {
            std::cout << "Goal reached at iteration " << i << "!" << std::endl;
            return smoothPath(reconstructPath(new_node));
        }
        
        if (dist_to_goal < goal_threshold_ * 2) {
            goal_reach_attempts++;
        }
    }

    // If we haven't reached the exact goal but have a close node, return that path
    if (best_goal_node && best_goal_dist < goal_threshold_ * 3) {
        std::cout << "Returning best available path (distance to goal: " 
                  << best_goal_dist << ")" << std::endl;
        auto path = reconstructPath(best_goal_node);
        path.push_back(goal);  // Add actual goal
        return smoothPath(path);
    }

    std::cout << "Failed to reach goal!" << std::endl;
    return {};
}

// Optimized collision checking
bool RRTStarPlanner::isCollisionFree(const Eigen::Vector2d& from, const Eigen::Vector2d& to) {
    if (!isPointInBounds(from) || !isPointInBounds(to)) {
        return false;
    }

    double dist = (to - from).norm();
    double step = robot_radius_ * 0.5;
    int steps = std::max(2, static_cast<int>(std::ceil(dist / step)));
    
    Eigen::Vector2d direction = (to - from) / steps;
    
    for (int i = 0; i <= steps; ++i) {
        Eigen::Vector2d point = from + direction * i;
        if (grid_.isCollision(point, robot_radius_)) {
            return false;
        }
    }
    
    return true;
}


bool RRTStarPlanner::isPointInBounds(const Eigen::Vector2d& point) const {
    bool inBounds = point.x() >= world_min_x_ && point.x() <= world_max_x_ &&
                   point.y() >= world_min_y_ && point.y() <= world_max_y_;
    if (!inBounds) {
        std::cout << "Point (" << point.x() << ", " << point.y() 
                  << ") is out of bounds. Bounds: [" << world_min_x_ << "," 
                  << world_max_x_ << "] x [" << world_min_y_ << "," 
                  << world_max_y_ << "]" << std::endl;
    }
    return inBounds;
}

Eigen::Vector3d RRTStarPlanner::sampleRandomPoint() {
    double x = std::uniform_real_distribution<>(world_min_x_, world_max_x_)(random_generator_);
    double y = std::uniform_real_distribution<>(world_min_y_, world_max_y_)(random_generator_);
    return Eigen::Vector3d(x, y, 0.0);
}

std::shared_ptr<RRTNode> RRTStarPlanner::findNearest(const Eigen::Vector3d& point) {
    if (nodes_.empty()) {
        return nullptr;
    }

    std::shared_ptr<RRTNode> nearest = nullptr;
    double min_dist = std::numeric_limits<double>::max();

    for (const auto& node : nodes_) {
        double dist = (node->position.head<2>() - point.head<2>()).norm();
        if (dist < min_dist) {
            min_dist = dist;
            nearest = node;
        }
    }

    return nearest;
}

std::vector<std::shared_ptr<RRTNode>> RRTStarPlanner::findNeighbors(const Eigen::Vector3d& point) {
    std::vector<std::shared_ptr<RRTNode>> neighbors;
    for (const auto& node : nodes_) {
        if ((node->position.head<2>() - point.head<2>()).norm() <= rewire_radius_) {
            neighbors.push_back(node);
        }
    }
    return neighbors;
}

double RRTStarPlanner::calculateCost(std::shared_ptr<RRTNode> from, std::shared_ptr<RRTNode> to) {
    return (to->position.head<2>() - from->position.head<2>()).norm();
}

void RRTStarPlanner::rewireNodes(std::shared_ptr<RRTNode> new_node,
                                const std::vector<std::shared_ptr<RRTNode>>& neighbors) {
    for (auto& neighbor : neighbors) {
        if (neighbor == new_node->parent) continue;

        double potential_cost = new_node->cost + calculateCost(new_node, neighbor);
        if (potential_cost < neighbor->cost &&
            isCollisionFree(new_node->position.head<2>(), neighbor->position.head<2>())) {
            
            if (neighbor->parent) {
                auto& children = neighbor->parent->children;
                children.erase(std::remove(children.begin(), children.end(), neighbor),
                             children.end());
            }
            
            double cost_change = potential_cost - neighbor->cost;
            neighbor->parent = new_node;
            neighbor->cost = potential_cost;
            new_node->children.push_back(neighbor);
            updateChildrenCosts(neighbor, cost_change);
        }
    }
}

void RRTStarPlanner::updateChildrenCosts(std::shared_ptr<RRTNode> node, double cost_change) {
    for (auto& child : node->children) {
        child->cost += cost_change;
        updateChildrenCosts(child, cost_change);
    }
}

std::vector<Eigen::Vector3d> RRTStarPlanner::reconstructPath(std::shared_ptr<RRTNode> goal_node) {
    std::vector<Eigen::Vector3d> path;
    auto current = goal_node;
    while (current) {
        path.push_back(current->position);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

Eigen::Vector3d RRTStarPlanner::steer(const Eigen::Vector3d& from, const Eigen::Vector3d& to) {
    Eigen::Vector3d diff = to - from;
    double dist = diff.head<2>().norm();
    
    if (dist <= step_size_) {
        return to;
    }
    
    return from + (diff * step_size_ / dist);
}