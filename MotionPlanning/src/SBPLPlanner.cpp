// SBPLPlanner.cpp

#include "SBPLPlanner.h"
#include <cmath>
#include <iostream>
#include <algorithm>

// Implementation of GridState equality operator
bool GridState::operator==(const GridState& other) const {
    return x == other.x && y == other.y && theta == other.theta;
}

// Implementation of GridStateHash
std::size_t GridStateHash::operator()(const GridState& state) const {
    // Combine the hash of x, y, and theta
    size_t hx = std::hash<int>()(state.x);
    size_t hy = std::hash<int>()(state.y);
    size_t ht = std::hash<int>()(state.theta);
    // Use a simple hash combining method
    return hx ^ (hy << 1) ^ (ht << 2);
}

// SBPLNode constructor
SBPLNode::SBPLNode(const GridState& state, double g_cost, double f_cost, std::shared_ptr<SBPLNode> parent)
    : state(state), g_cost(g_cost), f_cost(f_cost), parent(parent) {}

// Comparator for priority queue
bool CompareSBPLNode::operator()(const std::shared_ptr<SBPLNode>& a, const std::shared_ptr<SBPLNode>& b) const {
    return a->f_cost > b->f_cost; // Min-heap based on f_cost
}

// Constructor
SBPLPlanner::SBPLPlanner(const OccupancyGrid& grid, double robot_radius)
    : grid_(grid), robot_radius_(robot_radius), goal_threshold_(0.5), cost_threshold_(10000.0), resolution_(grid.resolution_m) {
    initializeMotionPrimitives();
}

// Initialize motion primitives
void SBPLPlanner::initializeMotionPrimitives() {
    double step_size = resolution_;
    motion_primitives_.clear();

    // 8-connected grid movements
    motion_primitives_.push_back(Eigen::Vector3d(step_size, 0.0, 0.0));
    motion_primitives_.push_back(Eigen::Vector3d(-step_size, 0.0, M_PI));
    motion_primitives_.push_back(Eigen::Vector3d(0.0, step_size, M_PI / 2));
    motion_primitives_.push_back(Eigen::Vector3d(0.0, -step_size, -M_PI / 2));
    motion_primitives_.push_back(Eigen::Vector3d(step_size, step_size, M_PI / 4));
    motion_primitives_.push_back(Eigen::Vector3d(-step_size, step_size, 3 * M_PI / 4));
    motion_primitives_.push_back(Eigen::Vector3d(-step_size, -step_size, -3 * M_PI / 4));
    motion_primitives_.push_back(Eigen::Vector3d(step_size, -step_size, -M_PI / 4));

    // Add smaller steps if necessary
    // For example, half-step movements
    double half_step = step_size / 2.0;
    motion_primitives_.push_back(Eigen::Vector3d(half_step, 0.0, 0.0));
    motion_primitives_.push_back(Eigen::Vector3d(-half_step, 0.0, M_PI));
    motion_primitives_.push_back(Eigen::Vector3d(0.0, half_step, M_PI / 2));
    motion_primitives_.push_back(Eigen::Vector3d(0.0, -half_step, -M_PI / 2));
}


// Heuristic function (Manhattan distance)
double SBPLPlanner::heuristic(const GridState& state, const GridState& goal_state) const {
    double dx = std::abs(state.x - goal_state.x);
    double dy = std::abs(state.y - goal_state.y);
    return (dx + dy) * resolution_;
}

// Main planning function
std::vector<Eigen::Vector3d> SBPLPlanner::getCollisionFreePath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal) {
    
    // Validate goal position and get alternative if needed
    Eigen::Vector3d validated_goal = validateGoal(goal, start);

    auto discretize = [this](const Eigen::Vector3d& pos) -> GridState {
        return {
            static_cast<int>(std::round(pos.x() / resolution_)),
            static_cast<int>(std::round(pos.y() / resolution_)),
            0 // Discretize orientation if needed
        };
    };

    auto continuousPosition = [this](const GridState& state) -> Eigen::Vector3d {
        return Eigen::Vector3d(
            state.x * resolution_,
            state.y * resolution_,
            0.0 // Handle orientation if needed
        );
    };

    std::priority_queue<std::shared_ptr<SBPLNode>, std::vector<std::shared_ptr<SBPLNode>>, CompareSBPLNode> open_list;
    std::unordered_map<GridState, double, GridStateHash> g_cost_map;
    std::unordered_set<GridState, GridStateHash> closed_set;
    node_map_.clear(); // Clear the node map at the start

    GridState start_state = discretize(start);
    GridState goal_state = discretize(validated_goal);

    double h_start = heuristic(start_state, goal_state);
    auto start_node = std::make_shared<SBPLNode>(start_state, 0.0, h_start);
    open_list.push(start_node);
    g_cost_map[start_state] = 0.0;
    node_map_[start_state] = start_node;

    while (!open_list.empty()) {
        auto current = open_list.top();
        open_list.pop();

        if (closed_set.find(current->state) != closed_set.end()) {
            continue;
        }
        closed_set.insert(current->state);

        // Get current position as Eigen::Vector3d
        Eigen::Vector3d current_pos = continuousPosition(current->state);

        // Check if goal is reached
        if ((current_pos.head<2>() - validated_goal.head<2>()).norm() <= goal_threshold_) {
            return reconstructPath(current);
        }

        for (const auto& primitive : motion_primitives_) {
            // Compute new position
            Eigen::Vector3d new_pos = current_pos + primitive;
            GridState new_state = discretize(new_pos);

            double movement_cost = primitive.head<2>().norm();
            double tentative_g_cost = current->g_cost + movement_cost;

            if (!isCollisionFree(new_pos.head<2>())) {
                continue;
            }

            auto it = g_cost_map.find(new_state);
            if (it != g_cost_map.end() && tentative_g_cost >= it->second) {
                continue;
            }

            double h_cost = heuristic(new_state, goal_state);
            double f_cost = tentative_g_cost + h_cost;

            auto neighbor = std::make_shared<SBPLNode>(new_state, tentative_g_cost, f_cost, current);
            open_list.push(neighbor);
            g_cost_map[new_state] = tentative_g_cost;
            node_map_[new_state] = neighbor;
        }
    }

    std::cerr << "Goal not reachable." << std::endl;
    return {};
}

// Reconstruct the path from the goal to the start
std::vector<Eigen::Vector3d> SBPLPlanner::reconstructPath(std::shared_ptr<SBPLNode> goal_node) {
    std::vector<Eigen::Vector3d> path;
    auto current = goal_node;

    while (current) {
        // Convert GridState back to continuous position
        Eigen::Vector3d pos;
        pos.x() = current->state.x * resolution_;
        pos.y() = current->state.y * resolution_;
        pos.z() = 0; // Set z if necessary
        path.push_back(pos);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}
