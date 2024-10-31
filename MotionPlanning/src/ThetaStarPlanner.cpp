#include "ThetaStarPlanner.h"
#include <iostream>
#include <cmath>

// Define neighbor offsets for 8-directional movement
const std::vector<Eigen::Vector3d> neighbor_offsets = {
    {1, 0, 0}, {0, 1, 0}, {-1, 0, 0}, {0, -1, 0},
    {1, 1, 0}, {-1, -1, 0}, {1, -1, 0}, {-1, 1, 0}
};

ThetaStarPlanner::ThetaStarPlanner(const OccupancyGrid& grid, double robot_radius)
    : AStarPlanner(grid, robot_radius) {}
std::vector<Eigen::Vector3d> ThetaStarPlanner::getCollisionFreePath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal) {
    std::cout << "Starting Theta* from " << start.transpose() << " to " << goal.transpose() << std::endl;

    if (grid_.isCollision(start.head<2>(), robot_radius_) || grid_.isCollision(goal.head<2>(), robot_radius_)) {
        std::cerr << "Start or goal is in collision" << std::endl;
        return {};
    }

    open_list_ = std::priority_queue<Node, std::vector<Node>, CompareNode>();
    g_cost_.clear();
    std::unordered_set<Eigen::Vector3d, Vector3dHash> closed_set;

    const double goal_vicinity_threshold = 0.3;  // Temporarily increase to make sure goal can be reached
    const double node_resolution = 0.05;  // Reduce node resolution

    Node start_node(start, 0.0, nullptr);
    open_list_.push(start_node);
    g_cost_[start] = 0.0;

    int iteration = 0;
    while (!open_list_.empty()) {
        Node current = open_list_.top();
        open_list_.pop();

        if (closed_set.find(current.position) != closed_set.end()) {
            continue;
        }

        iteration++;
        std::cout << "Iteration " << iteration << ": Expanding node at " << current.position.transpose() 
                  << " with cost " << current.cost << std::endl;

        if ((current.position.head<2>() - goal.head<2>()).norm() <= goal_vicinity_threshold) {
            std::cout << "Goal vicinity reached, reconstructing path..." << std::endl;
            return reconstructPath(current);
        }

        closed_set.insert(current.position);

        Node* parent = current.parent;
        bool has_direct_path = parent && (parent->position != current.position) &&
                               lineOfSight(parent->position.head<2>(), current.position.head<2>());

        if (has_direct_path) {
            current.cost = parent->cost + (parent->position.head<2>() - current.position.head<2>()).norm();
            current.parent = parent;
        } else {
            expandNeighbors(current, goal);
        }

        if (!has_direct_path) {
            open_list_.push(current);
        }
    }

    std::cerr << "Goal not reachable." << std::endl;
    return {};
}

// Simplify neighbor expansion for debugging
void ThetaStarPlanner::expandNeighbors(Node& current, const Eigen::Vector3d& goal) {
    std::cout << "Expanding neighbors for node at " << current.position.transpose() << std::endl;
    for (const auto& offset : neighbor_offsets) {
        Eigen::Vector3d neighbor_pos = current.position + offset * grid_.resolution_m;

        if ((neighbor_pos.head<2>() - goal.head<2>()).norm() >= (current.position.head<2>() - goal.head<2>()).norm()) {
            continue;  // Skip if neighbor moves further from goal
        }

        // Avoid redundant checks for now
        if (grid_.isCollision(neighbor_pos.head<2>(), robot_radius_)) {
      //      std::cout << "Collision detected at neighbor " << neighbor_pos.head<2>().transpose() << std::endl;
            continue;
        }

        double new_cost = current.cost + (current.position.head<2>() - neighbor_pos.head<2>()).norm();
        double heuristic_weight = 2;
        double heuristic = current.cost + (current.position.head<2>() - goal.head<2>()).norm() * heuristic_weight;
        double total_cost = new_cost + heuristic;

        Node neighbor(neighbor_pos, total_cost, &current);

        if (g_cost_.find(neighbor.position) == g_cost_.end() || new_cost < g_cost_[neighbor.position]) {
            g_cost_[neighbor.position] = new_cost;
            open_list_.push(neighbor);
            std::cout << "Neighbor at " << neighbor_pos.transpose() << " added to open list with cost " << total_cost << std::endl;
        }
    }
}


std::vector<Eigen::Vector3d> ThetaStarPlanner::reconstructPath(const Node& goal_node) {
    std::vector<Eigen::Vector3d> path;
    const Node* current = &goal_node;
    int max_iterations = 10000;
    int iteration = 0;

    std::cout << "Starting path reconstruction from goal..." << std::endl;

    // Collect all nodes from the goal to the start
    while (current) {
        path.push_back(current->position);
        std::cout << "Path node at " << current->position.transpose() << " with cost " << current->cost << std::endl;
        
        if (iteration++ > max_iterations) {
            std::cerr << "Error: Path reconstruction exceeded max iterations, potential cycle detected." << std::endl;
            break;
        }
        
        current = current->parent;
    }

    std::reverse(path.begin(), path.end());  // Reverse to start from initial node

    // Prune the path by removing collinear points
    path = prunePath(path);

    std::cout << "Path reconstruction complete. Path length after pruning: " << path.size() << std::endl;

    return path;
}

// Prune path by removing intermediate collinear points
std::vector<Eigen::Vector3d> ThetaStarPlanner::prunePath(const std::vector<Eigen::Vector3d>& path) {
    std::vector<Eigen::Vector3d> pruned_path;
    if (path.size() < 3) {
        return path;  // Nothing to prune if path is too short
    }

    pruned_path.push_back(path.front());
    for (size_t i = 1; i < path.size() - 1; ++i) {
        Eigen::Vector2d prev = pruned_path.back().head<2>();
        Eigen::Vector2d curr = path[i].head<2>();
        Eigen::Vector2d next = path[i + 1].head<2>();

        // Check if current point is roughly collinear with previous and next points
        if ((next - curr).normalized().dot((curr - prev).normalized()) < 0.999) {
            pruned_path.push_back(path[i]);  // Keep points where direction changes
        }
    }
    pruned_path.push_back(path.back());
    return pruned_path;
}

// Smooth path by replacing segments with direct lines if line-of-sight is clear
std::vector<Eigen::Vector3d> ThetaStarPlanner::smoothPathWithLineOfSight(const std::vector<Eigen::Vector3d>& path) {
    std::vector<Eigen::Vector3d> smoothed_path;
    size_t i = 0;
    while (i < path.size()) {
        smoothed_path.push_back(path[i]);
        size_t j = i + 1;

        // Look for the farthest point with line-of-sight to reduce segments
        while (j < path.size() && lineOfSight(path[i].head<2>(), path[j].head<2>())) {
            ++j;
        }

        // Move to the last point with a clear line of sight
        i = j - 1;
    }

    return smoothed_path;
}


bool ThetaStarPlanner::lineOfSight(const Eigen::Vector2d& start, const Eigen::Vector2d& end) {
    int x0 = static_cast<int>(std::round(start.x() / grid_.resolution_m));
    int y0 = static_cast<int>(std::round(start.y() / grid_.resolution_m));
    int x1 = static_cast<int>(std::round(end.x() / grid_.resolution_m));
    int y1 = static_cast<int>(std::round(end.y() / grid_.resolution_m));

    int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;

    while (x0 != x1 || y0 != y1) {
        if (grid_.isCollision(Eigen::Vector2d(x0 * grid_.resolution_m, y0 * grid_.resolution_m), robot_radius_)) {
//            std::cout << "Collision detected during line of sight check." << std::endl;
            return false;
        }

        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx) { err += dx; y0 += sy; }
    }
    return true;
}
