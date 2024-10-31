// STOMPPlanner.cpp
#include "STOMPPlanner.h"
#include <cmath>
#include <algorithm>
#include <iostream>

STOMPPlanner::STOMPPlanner(const OccupancyGrid& grid, double robot_radius)
    : grid_(grid), robot_radius_(robot_radius), 
      gen_(std::random_device{}()),
      noise_dist_(0.0, 1.0) {
    initializeParameters();
    precomputeMatrices();
}

void STOMPPlanner::initializeParameters() {
    n_timesteps_ = 100;
    n_noisy_trajectories_ = 50;
    max_iterations_ = 200;
    control_cost_weight_ = 0.001;    // Significantly reduced
    obstacle_cost_weight_ = 1.0;     // Base weight
    exploration_std_dev_ = 1.0;      // Increased for more exploration
    learning_rate_ = 0.5;            // Increased for faster learning
    min_clearance_ = robot_radius_ * 1.2;
    convergence_threshold_ = 0.1;    // Increased for easier convergence
}


void STOMPPlanner::precomputeMatrices() {
    // Construct finite differencing matrix for acceleration
    R_ = Eigen::MatrixXd::Zero(n_timesteps_, n_timesteps_);
    
    // Modified finite differencing for smoother trajectories
    for (int i = 0; i < n_timesteps_ - 2; ++i) {
        R_(i, i) = 1;
        R_(i, i+1) = -2;
        R_(i, i+2) = 1;
    }
    
    // Add velocity terms
    for (int i = 0; i < n_timesteps_ - 1; ++i) {
        R_(i, i) += 0.5;
        R_(i, i+1) += -0.5;
    }
    
    M_ = R_.transpose() * R_;
    
    // Better regularization
    Eigen::MatrixXd reg = Eigen::MatrixXd::Identity(n_timesteps_, n_timesteps_) * 1e-5;
    R_inv_ = (M_ + reg).inverse();
}


std::vector<Eigen::Vector3d> STOMPPlanner::initializeTrajectory(
    const Eigen::Vector3d& start, const Eigen::Vector3d& goal) {
    
    std::cout << "\nInitializing trajectory..." << std::endl;
    std::cout << "Grid dimensions: " << grid_.width << "x" << grid_.height << std::endl;
    std::cout << "Resolution: " << grid_.resolution_m << " m/cell" << std::endl;
    
    std::vector<Eigen::Vector3d> trajectory(n_timesteps_);
    
    // Try different initialization strategies
    std::vector<std::vector<Eigen::Vector3d>> candidate_trajectories;
    
    // 1. Direct line initialization
    for (int i = 0; i < n_timesteps_; ++i) {
        double alpha = static_cast<double>(i) / (n_timesteps_ - 1);
        trajectory[i] = (1.0 - alpha) * start + alpha * goal;
    }
    candidate_trajectories.push_back(trajectory);
    
    // 2. Add some curvature
    trajectory = candidate_trajectories[0];
    Eigen::Vector3d midpoint = (start + goal) * 0.5;
    Eigen::Vector3d perpendicular(-midpoint.y(), midpoint.x(), 0);
    perpendicular.normalize();
    
    for (int i = 1; i < n_timesteps_-1; ++i) {
        double alpha = static_cast<double>(i) / (n_timesteps_ - 1);
        double scale = std::sin(M_PI * alpha) * 2.0;
        trajectory[i] += perpendicular * scale;
    }
    candidate_trajectories.push_back(trajectory);
    
    // Choose the best initialization
    double best_cost = std::numeric_limits<double>::infinity();
    int best_idx = 0;
    
    for (size_t i = 0; i < candidate_trajectories.size(); ++i) {
        double cost = computeTotalCost(candidate_trajectories[i]);
        std::cout << "Initialization " << i << " cost: " << cost << std::endl;
        if (cost < best_cost) {
            best_cost = cost;
            best_idx = i;
        }
    }
    
    std::cout << "Selected initialization " << best_idx << std::endl;
    return candidate_trajectories[best_idx];
}


double STOMPPlanner::computeDistanceCost(const Eigen::Vector2d& point) {
    cv::Mat distance_map = grid_.computeDistanceMap();
    
    int x = static_cast<int>(std::round(point.x() / grid_.resolution_m));
    int y = grid_.height - static_cast<int>(std::round(point.y() / grid_.resolution_m));
    
    if (x < 0 || x >= grid_.width || y < 0 || y >= grid_.height) {
        std::cout << "Point (" << point.x() << ", " << point.y() << ") out of bounds!" << std::endl;
        return 1000.0;
    }
    
    double distance = distance_map.at<float>(y, x);
    std::cout << "Distance at point (" << point.x() << ", " << point.y() 
              << "): " << distance << std::endl;
    
    if (distance < min_clearance_) {
        return obstacle_cost_weight_ * (min_clearance_ - distance);
    }
    return 0.0;
}



double STOMPPlanner::computeControlCost(
    const std::vector<Eigen::Vector3d>& trajectory) {
    
    if (trajectory.size() < 3) return 0.0;
    
    double cost = 0.0;
    
    // Compute acceleration cost using finite differences
    for (size_t i = 0; i < trajectory.size() - 2; ++i) {
        Eigen::Vector3d acc = trajectory[i] - 2.0 * trajectory[i+1] + trajectory[i+2];
        double acc_norm = acc.squaredNorm();
        // Add small epsilon to avoid numerical issues
        cost += std::sqrt(acc_norm + 1e-10);
    }
    
    return control_cost_weight_ * cost;
}

double STOMPPlanner::computeObstacleCost(
    const std::vector<Eigen::Vector3d>& trajectory) {
    
    double cost = 0.0;
    
    for (const auto& point : trajectory) {
        cost += computeDistanceCost(point.head<2>());
    }
    
    return obstacle_cost_weight_ * cost;
}


double STOMPPlanner::computeTotalCost(const std::vector<Eigen::Vector3d>& trajectory) {
    // Control cost
    double control_cost = 0.0;
    for (size_t i = 0; i < trajectory.size() - 2; ++i) {
        Eigen::Vector3d acc = trajectory[i] - 2.0 * trajectory[i+1] + trajectory[i+2];
        control_cost += acc.squaredNorm();
    }
    control_cost *= control_cost_weight_;
    
    // Obstacle cost
    double obstacle_cost = 0.0;
    for (const auto& point : trajectory) {
        obstacle_cost += computeDistanceCost(point.head<2>());
    }
    
    std::cout << "Control cost: " << control_cost << ", Obstacle cost: " 
              << obstacle_cost << std::endl;
    
    return control_cost + obstacle_cost;
}


STOMPPlanner::TrajectoryParams STOMPPlanner::generateNoisyTrajectory(
    const std::vector<Eigen::Vector3d>& mean_trajectory) {
    
    TrajectoryParams noisy(n_timesteps_);
    
    // Generate noise with adaptive magnitude
    for (int i = 1; i < n_timesteps_-1; ++i) {
        // Scale noise based on distance to obstacles
        double clearance = grid_.computeDistanceMap().at<float>(
            grid_.height - static_cast<int>(std::round(mean_trajectory[i].y() / grid_.resolution_m)),
            static_cast<int>(std::round(mean_trajectory[i].x() / grid_.resolution_m)));
        
        double noise_scale = std::min(clearance / min_clearance_, 2.0);
        
        Eigen::Vector3d noise(
            noise_dist_(gen_) * exploration_std_dev_ * noise_scale,
            noise_dist_(gen_) * exploration_std_dev_ * noise_scale,
            0.0);
        
        noisy.points[i] = mean_trajectory[i] + noise;
    }
    
    // Keep endpoints fixed
    noisy.points[0] = mean_trajectory[0];
    noisy.points[n_timesteps_-1] = mean_trajectory[n_timesteps_-1];
    
    // Smooth the noisy trajectory
    for (int i = 1; i < n_timesteps_-1; ++i) {
        noisy.points[i] = 0.25 * noisy.points[i-1] + 
                         0.5 * noisy.points[i] + 
                         0.25 * noisy.points[i+1];
    }
    
    noisy.cost = computeTotalCost(noisy.points);
    return noisy;
}

void STOMPPlanner::updateTrajectory(
    std::vector<Eigen::Vector3d>& trajectory,
    const std::vector<TrajectoryParams>& noisy_trajectories) {
    
    // Add the current trajectory to the set with zero noise
    std::vector<TrajectoryParams> all_trajectories = noisy_trajectories;
    TrajectoryParams current_traj(n_timesteps_);
    current_traj.points = trajectory;
    current_traj.cost = computeTotalCost(trajectory);
    all_trajectories.push_back(current_traj);
    
    // Sort trajectories by cost
    std::sort(all_trajectories.begin(), all_trajectories.end(),
              [](const TrajectoryParams& a, const TrajectoryParams& b) {
                  return a.cost < b.cost;
              });
    
    // Take weighted average of top trajectories
    std::vector<Eigen::Vector3d> new_trajectory = trajectory;
    const int top_k = std::max(3, static_cast<int>(all_trajectories.size() / 4));
    
    for (int i = 1; i < n_timesteps_-1; ++i) {
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        double weight_sum = 0.0;
        
        for (int j = 0; j < top_k; ++j) {
            double weight = std::exp(-j);  // Exponential weighting
            sum += weight * all_trajectories[j].points[i];
            weight_sum += weight;
        }
        
        new_trajectory[i] = sum / weight_sum;
    }
    
    // Update with learning rate
    for (int i = 1; i < n_timesteps_-1; ++i) {
        trajectory[i] = trajectory[i] + learning_rate_ * (new_trajectory[i] - trajectory[i]);
    }
    
    std::cout << "Updated trajectory cost: " << computeTotalCost(trajectory) << std::endl;
}


bool STOMPPlanner::optimizeTrajectory(std::vector<Eigen::Vector3d>& trajectory) {
    double prev_cost = std::numeric_limits<double>::max();
    int stagnant_iterations = 0;
    
    for (int iter = 0; iter < max_iterations_; ++iter) {
        // Generate noisy trajectories
        std::vector<TrajectoryParams> noisy_trajectories;
        for (int i = 0; i < n_noisy_trajectories_; ++i) {
            auto noisy = generateNoisyTrajectory(trajectory);
            if (std::isfinite(noisy.cost)) {
                noisy_trajectories.push_back(noisy);
            }
        }
        
        if (noisy_trajectories.empty()) {
            std::cout << "No valid noisy trajectories generated at iteration " << iter << std::endl;
            continue;
        }
        
        // Update trajectory
        updateTrajectory(trajectory, noisy_trajectories);
        
        // Check convergence
        double current_cost = computeTotalCost(trajectory);
        
        if (std::isfinite(current_cost)) {
            if (std::abs(current_cost - prev_cost) < convergence_threshold_) {
                stagnant_iterations++;
                if (stagnant_iterations > 5) {
                    std::cout << "STOMP converged after " << iter << " iterations" << std::endl;
                    return true;
                }
            } else {
                stagnant_iterations = 0;
            }
            
            if (iter % 10 == 0) {
                std::cout << "Iteration " << iter << ", Cost: " << current_cost << std::endl;
            }
            
            prev_cost = current_cost;
        } else {
            std::cout << "Invalid cost at iteration " << iter << std::endl;
        }
    }
    
    return false;
}

std::vector<Eigen::Vector3d> STOMPPlanner::getCollisionFreePath(
    const Eigen::Vector3d& start, const Eigen::Vector3d& goal) {
    
    std::cout << "Starting STOMP planning..." << std::endl;
    std::cout << "Start: (" << start.x() << ", " << start.y() << ")" << std::endl;
    std::cout << "Goal: (" << goal.x() << ", " << goal.y() << ")" << std::endl;
    
    // Initialize with multiple candidate trajectories
    auto initial_trajectory = initializeTrajectory(start, goal);
    double best_cost = std::numeric_limits<double>::infinity();
    std::vector<Eigen::Vector3d> best_trajectory;
    
    // Try multiple initializations
    for (int init = 0; init < 3; init++) {
        auto trajectory = initial_trajectory;
        if (init > 0) {
            // Add some random perturbation for subsequent attempts
            for (size_t i = 1; i < trajectory.size() - 1; i++) {
                trajectory[i].x() += noise_dist_(gen_) * 0.5;
                trajectory[i].y() += noise_dist_(gen_) * 0.5;
            }
        }
        
        if (optimizeTrajectory(trajectory)) {
            double cost = computeTotalCost(trajectory);
            if (cost < best_cost) {
                best_cost = cost;
                best_trajectory = trajectory;
            }
        }
    }
    
    if (!best_trajectory.empty()) {
        std::cout << "STOMP planning succeeded with cost: " << best_cost << std::endl;
        return best_trajectory;
    } else {
        std::cout << "STOMP planning failed!" << std::endl;
        return std::vector<Eigen::Vector3d>();
    }
}