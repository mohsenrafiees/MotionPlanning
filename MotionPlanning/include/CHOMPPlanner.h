// CHOMPPlanner.h
#ifndef CHOMP_PLANNER_H
#define CHOMP_PLANNER_H

#include "BasePlanner.h"
#include "OccupancyGrid.h"
#include <vector>
#include <Eigen/Dense>
#include <Eigen/StdVector>

class CHOMPPlanner : public BasePlanner {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CHOMPPlanner(const OccupancyGrid& grid, double robot_radius);

    // Required by BasePlanner
    std::vector<Eigen::Vector3d> getCollisionFreePath(
        const Eigen::Vector3d& start, const Eigen::Vector3d& goal) override;

protected:
    // Override the pure virtual method from BasePlanner
    virtual bool isCollisionFree(const Eigen::Vector2d& position) override {
        return !grid_.isCollision(position, robot_radius_);
    }

private:
    struct TrajectoryPoint {
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d acceleration;
    };

    const OccupancyGrid& grid_;
    double robot_radius_;
    
    // CHOMP parameters
    int n_points_;              // Number of trajectory points
    double learning_rate_;      // Step size for gradient descent
    double smoothness_cost_weight_;
    double obstacle_cost_weight_;
    double ridge_factor_;       // Regularization factor
    int max_iterations_;
    double convergence_threshold_;
    double initial_learning_rate_;

    // Precomputed matrices for finite differencing
    Eigen::MatrixXd A_;  // Finite differencing matrix
    Eigen::MatrixXd K_;  // Smoothness cost matrix

    // Helper methods
    void applyGradientClamping(Eigen::MatrixXd& gradient);
    void updateLearningRate(int iteration);
    Eigen::MatrixXd computeGradient(const std::vector<Eigen::Vector3d>& trajectory);
    double computeMaxObstacleCost(const std::vector<Eigen::Vector3d>& trajectory);
    double computeCost(const std::vector<Eigen::Vector3d>& trajectory);
    void initializeParameters();
    void precomputeMatrices();
    std::vector<Eigen::Vector3d> initializeTrajectory(
        const Eigen::Vector3d& start, const Eigen::Vector3d& goal);
    
    // Cost computation
    double computeSmoothnessCost(const std::vector<Eigen::Vector3d>& trajectory);
    double computeObstacleCost(const std::vector<Eigen::Vector3d>& trajectory);
    double computeCollisionPotential(const Eigen::Vector2d& point);
    
    // Gradient computation
    std::vector<Eigen::Vector3d> computeSmoothnessCostGradient(
        const std::vector<Eigen::Vector3d>& trajectory);
    std::vector<Eigen::Vector3d> computeObstacleCostGradient(
        const std::vector<Eigen::Vector3d>& trajectory);
    
    // Optimization
    std::vector<Eigen::Vector3d> optimizeTrajectory(
    const std::vector<Eigen::Vector3d>& initial_trajectory);
    std::vector<Eigen::Vector3d> updateTrajectory(std::vector<Eigen::Vector3d>& trajectory, const Eigen::MatrixXd& gradient);


    // Utility functions
    bool isStateValid(const Eigen::Vector3d& state);
    Eigen::Vector2d computeDistanceGradient(const Eigen::Vector2d& point);
};

#endif // CHOMP_PLANNER_H

