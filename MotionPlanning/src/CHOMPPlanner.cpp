// CHOMPPlanner.cpp
#include "CHOMPPlanner.h"
#include <cmath>
#include <iostream>

CHOMPPlanner::CHOMPPlanner(const OccupancyGrid& grid, double robot_radius)
    : grid_(grid), robot_radius_(robot_radius) {
    initializeParameters();
    precomputeMatrices();
}

void CHOMPPlanner::initializeParameters() {
    n_points_ = 100;  // Number of waypoints in trajectory
    initial_learning_rate_ = 0.05;
    learning_rate_ = initial_learning_rate_;  // Dynamic learning rate
    smoothness_cost_weight_ = 0.1;
    obstacle_cost_weight_ = 5.0;  // Increased obstacle cost weight
    ridge_factor_ = 0.0001;
    max_iterations_ = 500;  // Increased maximum iterations
    convergence_threshold_ = 0.0001;  // Reduced threshold for better convergence
}
void CHOMPPlanner::applyGradientClamping(Eigen::MatrixXd& gradient) {
    double max_gradient = 10.0;  // Clamping threshold
    for (int i = 0; i < gradient.rows(); ++i) {
        gradient.row(i) = gradient.row(i).cwiseMin(max_gradient).cwiseMax(-max_gradient);
    }
}
void CHOMPPlanner::updateLearningRate(int iteration) {
    if (iteration > 0 && iteration % 50 == 0) {  // Annealing learning rate
        learning_rate_ *= 0.9;  // Reduce by 10% every 50 iterations
        learning_rate_ = std::max(learning_rate_, 1e-4);  // Lower bound for learning rate
    }
}

void CHOMPPlanner::precomputeMatrices() {
    // Initialize finite differencing matrix A
    A_ = Eigen::MatrixXd::Zero(n_points_, n_points_);
    for (int i = 0; i < n_points_; i++) {
        if (i > 0) A_(i, i-1) = -1;
        A_(i, i) = 1;
    }

    // Compute K = A^T A for smoothness objective
    K_ = A_.transpose() * A_;
}

std::vector<Eigen::Vector3d> CHOMPPlanner::initializeTrajectory(
    const Eigen::Vector3d& start, const Eigen::Vector3d& goal) {
    
    std::vector<Eigen::Vector3d> trajectory(n_points_);
    
    // Initialize with straight line
    for (int i = 0; i < n_points_; i++) {
        double alpha = static_cast<double>(i) / (n_points_ - 1);
        trajectory[i] = start + alpha * (goal - start);
    }
    return trajectory;
}

double CHOMPPlanner::computeSmoothnessCost(const std::vector<Eigen::Vector3d>& trajectory) {
    double cost = 0.0;
    
    // Compute velocities using finite differences
    for (size_t i = 0; i < trajectory.size() - 1; ++i) {
        Eigen::Vector3d vel = trajectory[i+1] - trajectory[i];
        cost += vel.squaredNorm();
    }
    
    return 0.5 * smoothness_cost_weight_ * cost;
}

double CHOMPPlanner::computeCollisionPotential(const Eigen::Vector2d& point) {
    cv::Mat distance_map = grid_.computeDistanceMap();
    
    // Convert point to grid coordinates
    int x = static_cast<int>(std::round(point.x() / grid_.resolution_m));
    int y = grid_.height - static_cast<int>(std::round(point.y() / grid_.resolution_m));
    
    if (x < 0 || x >= grid_.width || y < 0 || y >= grid_.height) {
        return std::numeric_limits<double>::max();
    }
    
    double distance = distance_map.at<float>(y, x);
    
    // Parameters for potential field
    const double safety_margin = robot_radius_ * 1.5;  // Increased safety margin
    const double potential_cutoff = robot_radius_ * 3.0;  // Distance at which potential goes to zero
    
    // Compute potential using a smoother function
    if (distance < potential_cutoff) {
        if (distance < robot_radius_) {
            // High cost inside or very close to obstacles
            return 10.0 + 0.5 * (distance - robot_radius_) * (distance - robot_radius_);
        } else if (distance < safety_margin) {
            // Quadratic cost in safety margin region
            double normalized_dist = (distance - robot_radius_) / (safety_margin - robot_radius_);
            return 0.5 * (1.0 - normalized_dist) * (1.0 - normalized_dist);
        } else {
            // Smooth decay to zero
            double normalized_dist = (distance - safety_margin) / (potential_cutoff - safety_margin);
            return 0.1 * (1.0 - normalized_dist) * (1.0 - normalized_dist);
        }
    }
    
    return 0.0;
}


double CHOMPPlanner::computeObstacleCost(const std::vector<Eigen::Vector3d>& trajectory) {
    double cost = 0.0;
    
    for (const auto& point : trajectory) {
        cost += computeCollisionPotential(point.head<2>());
    }
    
    return obstacle_cost_weight_ * cost;
}

Eigen::Vector2d CHOMPPlanner::computeDistanceGradient(const Eigen::Vector2d& point) {
    const double eps = 0.01;
    Eigen::Vector2d gradient;
    
    // Compute gradient using central differences
    gradient.x() = (computeCollisionPotential(point + Eigen::Vector2d(eps, 0)) -
                   computeCollisionPotential(point - Eigen::Vector2d(eps, 0))) / (2 * eps);
    gradient.y() = (computeCollisionPotential(point + Eigen::Vector2d(0, eps)) -
                   computeCollisionPotential(point - Eigen::Vector2d(0, eps))) / (2 * eps);
    
    return gradient;
}

std::vector<Eigen::Vector3d> CHOMPPlanner::computeSmoothnessCostGradient(
    const std::vector<Eigen::Vector3d>& trajectory) {
    
    std::vector<Eigen::Vector3d> gradient(trajectory.size());
    
    // Apply K matrix to trajectory
    for (size_t i = 0; i < trajectory.size(); ++i) {
        gradient[i].setZero();
        for (size_t j = 0; j < trajectory.size(); ++j) {
            gradient[i] += K_(i, j) * trajectory[j];
        }
    }
    
    return gradient;
}

std::vector<Eigen::Vector3d> CHOMPPlanner::computeObstacleCostGradient(
    const std::vector<Eigen::Vector3d>& trajectory) {
    
    std::vector<Eigen::Vector3d> gradient(trajectory.size());
    
    for (size_t i = 0; i < trajectory.size(); ++i) {
        Eigen::Vector2d grad_2d = computeDistanceGradient(trajectory[i].head<2>());
        gradient[i] = Eigen::Vector3d(grad_2d.x(), grad_2d.y(), 0.0);
    }
    
    return gradient;
}

std::vector<Eigen::Vector3d> CHOMPPlanner::optimizeTrajectory(
    const std::vector<Eigen::Vector3d>& initial_trajectory) {

    std::vector<Eigen::Vector3d> trajectory = initial_trajectory;
    for (int iteration = 0; iteration < max_iterations_; iteration++) {
        Eigen::MatrixXd gradient = computeGradient(trajectory);

        applyGradientClamping(gradient);  // Clamp gradient to prevent overflow
        updateLearningRate(iteration);    // Update learning rate dynamically

        trajectory = updateTrajectory(trajectory, gradient);

        double cost = computeCost(trajectory);
        double max_obstacle_cost = computeMaxObstacleCost(trajectory);

        std::cout << "Iteration " << iteration << ", Cost: " << cost
                  << ", Max Obstacle Cost: " << max_obstacle_cost << std::endl;

        if (cost < convergence_threshold_) {
            std::cout << "Converged at iteration " << iteration << std::endl;
            break;
        }
    }
    return trajectory;
}
double CHOMPPlanner::computeMaxObstacleCost(const std::vector<Eigen::Vector3d>& trajectory) {
    double max_cost = 0.0;
    for (const auto& point : trajectory) {
        max_cost = std::max(max_cost, grid_.getObstacleCost(point.x(), point.y()));
    }
    return max_cost;
}

// CHOMPPlanner.cpp
std::vector<Eigen::Vector3d> CHOMPPlanner::updateTrajectory(std::vector<Eigen::Vector3d>& trajectory, const Eigen::MatrixXd& gradient) {
    std::vector<Eigen::Vector3d> new_trajectory = trajectory;
    for (int i = 1; i < n_points_ - 1; i++) {  // Skip start and end points
        new_trajectory[i] -= learning_rate_ * gradient.row(i).transpose();
    }
    return new_trajectory;
}


double CHOMPPlanner::computeCost(const std::vector<Eigen::Vector3d>& trajectory) {
    double cost = 0.0;

    // Smoothness cost
    for (int i = 1; i < n_points_ - 1; i++) {
        Eigen::Vector3d diff = trajectory[i + 1] - 2 * trajectory[i] + trajectory[i - 1];
        cost += smoothness_cost_weight_ * diff.squaredNorm();
    }

    // Obstacle cost
    for (const auto& point : trajectory) {
        cost += obstacle_cost_weight_ * grid_.getObstacleCost(point.x(), point.y());
    }

    return cost;
}

Eigen::MatrixXd CHOMPPlanner::computeGradient(const std::vector<Eigen::Vector3d>& trajectory) {
    Eigen::MatrixXd gradient = Eigen::MatrixXd::Zero(n_points_, 3);

    // Compute smoothness gradient
    for (int i = 1; i < n_points_ - 1; i++) {
        gradient.row(i) += 2.0 * smoothness_cost_weight_ * K_ * trajectory[i];
    }

    // Compute obstacle gradient (simple Euclidean distance from obstacles)
    for (int i = 0; i < n_points_; i++) {
        double obstacle_cost = grid_.getObstacleCost(trajectory[i].x(), trajectory[i].y());
        gradient.row(i) += obstacle_cost_weight_ * obstacle_cost * trajectory[i];
    }

    return gradient;
}

std::vector<Eigen::Vector3d> CHOMPPlanner::getCollisionFreePath(
    const Eigen::Vector3d& start, const Eigen::Vector3d& goal) {
    
    std::cout << "Starting CHOMP planning..." << std::endl;
    std::cout << "Start: (" << start.x() << ", " << start.y() << ")" << std::endl;
    std::cout << "Goal: (" << goal.x() << ", " << goal.y() << ")" << std::endl;

    // Validate goal position and get alternative if needed
    Eigen::Vector3d validated_goal = validateGoal(goal, start);
    
    // Initialize trajectory
    auto trajectory = initializeTrajectory(start, validated_goal);
    
    // Optimize trajectory
    trajectory = optimizeTrajectory(trajectory);
    
    if (trajectory.size() < 1) {
        std::cout << "CHOMP planning succeeded!" << std::endl;
        return trajectory;
    } else {
        std::cout << "CHOMP planning failed!" << std::endl;
        return std::vector<Eigen::Vector3d>();
    }
}