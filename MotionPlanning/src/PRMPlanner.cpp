#include "PRMPlanner.h"
#include <algorithm>
#include <queue>
#include <cmath>
#include <random>

PRMPlanner::PRMPlanner(const OccupancyGrid& grid, double robot_radius,
                       int num_samples, double connection_radius, int k_nearest)
    : grid_(grid), robot_radius_(robot_radius), num_samples_(num_samples),
      connection_radius_(connection_radius), k_nearest_(k_nearest),
      rng_(std::random_device{}()), roadmap_built_(false) {}

void PRMPlanner::buildRoadmap() {
    // Sample random points
    for (int i = 0; i < num_samples_; ++i) {
        Eigen::Vector3d point = sampleRandomPoint();
        if (!grid_.isOccupied(point)) {
            nodes_.push_back(std::make_shared<PRMNode>(point));
        }
    }

    // Connect nodes
    for (auto node : nodes_) {
        connectNode(node);
    }

    roadmap_built_ = true;
}

Eigen::Vector3d PRMPlanner::sampleRandomPoint() {
    std::uniform_real_distribution<double> dist_x(grid_.min_x(), grid_.max_x());
    std::uniform_real_distribution<double> dist_y(grid_.min_y(), grid_.max_y());
    std::uniform_real_distribution<double> dist_z(grid_.min_z(), grid_.max_z());
    return Eigen::Vector3d(dist_x(rng_), dist_y(rng_), dist_z(rng_));
}

std::vector<std::shared_ptr<PRMNode>> PRMPlanner::findNearestNodes(
    const Eigen::Vector3d& point, int k) {
    std::vector<std::shared_ptr<PRMNode>> nearest_nodes;
    std::vector<double> distances;
    for (auto node : nodes_) {
        double dist = (node->position - point).norm();
        if (dist <= connection_radius_) {
            nearest_nodes.push_back(node);
            distances.push_back(dist);
        }
    }

    if (nearest_nodes.size() > k) {
        std::vector<size_t> indices(distances.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::partial_sort(indices.begin(), indices.begin() + k, indices.end(),
                          [&](size_t i1, size_t i2) { return distances[i1] < distances[i2]; });
        nearest_nodes.resize(k);
        for (size_t i = 0; i < k; ++i) {
            nearest_nodes[i] = nearest_nodes[indices[i]];
        }
    }

    return nearest_nodes;
}

bool PRMPlanner::isPathCollisionFree(const Eigen::Vector3d& from, const Eigen::Vector3d& to) {
    Eigen::Vector3d direction = to - from;
    double distance = direction.norm();
    direction.normalize();

    for (double step = 0; step <= distance; step += grid_.resolution()) {
        Eigen::Vector3d point = from + step * direction;
        if (grid_.isOccupied(point, robot_radius_)) {
            return false;
        }
    }

    return true;
}

void PRMPlanner::connectNode(std::shared_ptr<PRMNode> node) {
    std::vector<std::shared_ptr<PRMNode>> nearest_nodes = findNearestNodes(node->position, k_nearest_);

    for (auto neighbor : nearest_nodes) {
        if (isPathCollisionFree(node->position, neighbor->position)) {
            double cost = (node->position - neighbor->position).norm();
            node->neighbors.emplace_back(neighbor, cost);
            neighbor->neighbors.emplace_back(node, cost);
        }
    }
}

std::vector<Eigen::Vector3d> PRMPlanner::findPathAstar(
    std::shared_ptr<PRMNode> start_node, std::shared_ptr<PRMNode> goal_node) {
    struct AstarNode {
        std::shared_ptr<PRMNode> node;
        double g_cost;
        double f_cost;
        std::shared_ptr<AstarNode> parent;

        AstarNode(std::shared_ptr<PRMNode> n, double g, double f, std::shared_ptr<AstarNode> p)
            : node(n), g_cost(g), f_cost(f), parent(p) {}
    };

    auto compare_f_cost = [](const std::shared_ptr<AstarNode>& a, const std::shared_ptr<AstarNode>& b) {
        return a->f_cost > b->f_cost;
    };

    std::priority_queue<std::shared_ptr<AstarNode>, std::vector<std::shared_ptr<AstarNode>>, decltype(compare_f_cost)> open_set(compare_f_cost);

    std::unordered_map<std::shared_ptr<PRMNode>, std::shared_ptr<AstarNode>> node_to_astar_node;

    auto start_astar_node = std::make_shared<AstarNode>(start_node, 0, 0, nullptr);
    open_set.push(start_astar_node);
    node_to_astar_node[start_node] = start_astar_node;

    while (!open_set.empty()) {
        auto current_astar_node = open_set.top();
        open_set.pop();

        if (current_astar_node->node == goal_node) {
            std::vector<Eigen::Vector3d> path;
            while (current_astar_node != nullptr) {
                path.push_back(current_astar_node->node->position);
                current_astar_node = current_astar_node->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (const auto& neighbor : current_astar_node->node->neighbors) {
            double tentative_g_cost = current_astar_node->g_cost + neighbor.second;

            if (node_to_astar_node.find(neighbor.first) == node_to_astar_node.end()) {
                double h_cost = (neighbor.first->position - goal_node->position).norm();
                double f_cost = tentative_g_cost + h_cost;
                auto neighbor_astar_node = std::make_shared<AstarNode>(neighbor.first, tentative_g_cost, f_cost, current_astar_node);
                open_set.push(neighbor_astar_node);
                node_to_astar_node[neighbor.first] = neighbor_astar_node;
            } else if (tentative_g_cost < node_to_astar_node[neighbor.first]->g_cost) {
                auto neighbor_astar_node = node_to_astar_node[neighbor.first];
                neighbor_astar_node->parent = current_astar_node;
                neighbor_astar_node->g_cost = tentative_g_cost;
                neighbor_astar_node->f_cost = tentative_g_cost + (neighbor.first->position - goal_node->position).norm();
            }
        }
    }

    return {};
}

std::vector<Eigen::Vector3d> PRMPlanner::getCollisionFreePath(
    const Eigen::Vector3d& start, const Eigen::Vector3d& goal) {
    if (!roadmap_built_) {
        buildRoadmap();
    }

    auto start_node = std::make_shared<PRMNode>(start);
    auto goal_node = std::make_shared<PRMNode>(goal);

    connectNode(start_node);
    connectNode(goal_node);

    return findPathAstar(start_node, goal_node);
}