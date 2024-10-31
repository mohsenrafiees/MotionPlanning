// STOMPPlanner.h
#ifndef STOMP_PLANNER_H
#define STOMP_PLANNER_H

#include "BasePlanner.h"
#include "OccupancyGrid.h"
#include <vector>
#include <random>
#include <Eigen/Dense>
#include <Eigen/StdVector>

class STOMPPlanner : public BasePlanner {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    STOMPPlanner(const OccupancyGrid& grid, double robot_radius);

    std::vector<Eigen::Vector3d> getCollisionFreePath(
        const Eigen::Vector3d& start, const Eigen::Vector3d& goal) override;
protected:
    // Override the pure virtual method from BasePlanner
    virtual bool isCollisionFree(const Eigen::Vector2d& position) override {
        return !grid_.isCollision(position, robot_radius_);
    }

private:
    struct TrajectoryParams {
        std::vector<Eigen::Vector3d> points;
        double cost;
        
        TrajectoryParams(size_t size) : points(size), cost(0.0) {}
    };

    const OccupancyGrid& grid_;
    double robot_radius_;
    
    // STOMP parameters
    int n_timesteps_;        // Number of trajectory points
    int n_noisy_trajectories_; // Number of noisy trajectories per iteration
    int max_iterations_;     // Maximum optimization iterations
    double control_cost_weight_;
    double obstacle_cost_weight_;
    double exploration_std_dev_;  // Standard deviation for noise
    double learning_rate_;
    double min_clearance_;   // Minimum distance from obstacles
    double convergence_threshold_;

    // Precomputed matrices
    Eigen::MatrixXd R_;      // Control cost matrix
    Eigen::MatrixXd R_inv_;  // Inverse of control cost matrix
    Eigen::MatrixXd M_;      // Smoothing matrix

    // Random number generation
    std::mt19937 gen_;
    std::normal_distribution<double> noise_dist_;

    // Initialize parameters and matrices
    void initializeParameters();
    void precomputeMatrices();

    // Trajectory operations
    std::vector<Eigen::Vector3d> initializeTrajectory(
        const Eigen::Vector3d& start, const Eigen::Vector3d& goal);
    TrajectoryParams generateNoisyTrajectory(
        const std::vector<Eigen::Vector3d>& mean_trajectory);
    void updateTrajectory(std::vector<Eigen::Vector3d>& trajectory,
                         const std::vector<TrajectoryParams>& noisy_trajectories);
    
    // Cost computation
    double computeControlCost(const std::vector<Eigen::Vector3d>& trajectory);
    double computeObstacleCost(const std::vector<Eigen::Vector3d>& trajectory);
    double computeTotalCost(const std::vector<Eigen::Vector3d>& trajectory);
    
    // Optimization
    bool optimizeTrajectory(std::vector<Eigen::Vector3d>& trajectory);
    
    // Utility functions
    bool isStateValid(const Eigen::Vector3d& state);
    double computeDistanceCost(const Eigen::Vector2d& point);
};

#endif // STOMP_PLANNER_H

