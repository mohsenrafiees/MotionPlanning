#ifndef NODE_H
#define NODE_H

#include <Eigen/Dense>
#include <memory>

class Node {
public:
    double cost;  
    Eigen::Matrix<double, 3, 1> position;  
    Node* parent;  

    Node(const Eigen::Matrix<double, 3, 1>& pos, double cost_val = 0.0, Node* parent_node = nullptr)
        : cost(cost_val), position(pos), parent(parent_node) {}

    bool operator<(const Node& other) const {
        return this->cost > other.cost;  
    }
};

// Comparator for priority_queue to prioritize by cost
struct CompareNode {
    bool operator()(const Node& a, const Node& b) const {
        return a.cost > b.cost;
    }
};

#endif  // NODE_H
