#include "PathProfiler.h"
#include "plotting_utils.h"
#include "matplotlibcpp.h"
#include <numeric>
#include <cmath>
#include <fstream>
#include <iomanip>

namespace plt = matplotlibcpp;

PathProfiler::PathProfiler(const OccupancyGrid& grid, double robot_radius)
        : grid_(grid), robot_radius_(robot_radius) {}

// Helper function implementations
double PathProfiler::computePathLength(const std::vector<Eigen::Vector3d>& path) noexcept {
    if (path.size() < 2) return 0.0;
    
    double length = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        length += (path[i].head<2>() - path[i-1].head<2>()).norm();
    }
    return length;
}

double PathProfiler::computeAverageCurvature(const std::vector<Eigen::Vector3d>& path) noexcept {
    if (path.size() < 3) return 0.0;
    
    double total_curvature = 0.0;
    int count = 0;
    
    for (size_t i = 1; i < path.size() - 1; ++i) {
        Eigen::Vector2d v1 = path[i].head<2>() - path[i-1].head<2>();
        Eigen::Vector2d v2 = path[i+1].head<2>() - path[i].head<2>();
        
        if (v1.norm() < 1e-6 || v2.norm() < 1e-6) continue;
        
        double dot = v1.dot(v2) / (v1.norm() * v2.norm());
        // Clamp dot product to [-1, 1] to avoid numerical issues
        dot = std::max(-1.0, std::min(1.0, dot));
        double angle = std::acos(dot);
        total_curvature += angle;
        count++;
    }
    
    return count > 0 ? total_curvature / count : 0.0;
}

double PathProfiler::computeMinClearance(const Eigen::Vector2d& position) noexcept {
    cv::Mat distance_map = grid_.computeDistanceMap();
    
    int x = static_cast<int>(std::round(position.x() / grid_.resolution_m));
    int y = static_cast<int>(std::round(position.y() / grid_.resolution_m));
    
    // Ensure indices are within bounds
    x = std::clamp(x, 0, grid_.width - 1);
    y = std::clamp(y, 0, grid_.height - 1);
    
    return distance_map.at<float>(y, x);
}

double PathProfiler::computeHeadingChange(double h1, double h2) noexcept{
    double diff = h2 - h1;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return diff;
}

double PathProfiler::computePathSmoothness(const std::vector<Eigen::Vector3d>& path) noexcept{
    if (path.size() < 3) return 1.0;
    
    double total_angle_change = 0.0;
    int count = 0;
    
    for (size_t i = 1; i < path.size() - 1; ++i) {
        Eigen::Vector2d v1 = path[i].head<2>() - path[i-1].head<2>();
        Eigen::Vector2d v2 = path[i+1].head<2>() - path[i].head<2>();
        
        if (v1.norm() < 1e-6 || v2.norm() < 1e-6) continue;
        
        double dot = v1.dot(v2) / (v1.norm() * v2.norm());
        // Clamp dot product to [-1, 1] to avoid numerical issues
        dot = std::max(-1.0, std::min(1.0, dot));
        double angle_change = std::acos(dot);
        total_angle_change += angle_change;
        count++;
    }
    
    if (count == 0) return 1.0;
    double avg_angle_change = total_angle_change / count;
    return 1.0 / (1.0 + avg_angle_change);  // Normalize to [0,1]
}

double PathProfiler::computeSafetyScore(const std::vector<Eigen::Vector3d>& path) noexcept {
    if (path.empty()) return 0.0;
    
    double min_clearance = std::numeric_limits<double>::max();
    double total_clearance = 0.0;
    
    for (const auto& point : path) {
        double clearance = computeMinClearance(point.head<2>());
        min_clearance = std::min(min_clearance, clearance);
        total_clearance += clearance;
    }
    
    double avg_clearance = total_clearance / path.size();
    double clearance_score = avg_clearance / (robot_radius_ * 5.0);  // Normalize clearance
    
    double smoothness_score = computePathSmoothness(path);
    
    // Combine clearance and smoothness for safety score
    return 0.7 * clearance_score + 0.3 * smoothness_score;
}

double PathProfiler::computeTrajectorySmoothness(
    const std::vector<TrajectoryPoint>& trajectory) noexcept {
    
    if (trajectory.size() < 3) return 1.0;
    
    try {
        double total_jerk = 0.0;
        size_t valid_samples = 0;
        
        for (size_t i = 1; i < trajectory.size() - 1; ++i) {
            double dt = trajectory[i+1].time - trajectory[i].time;
            if (dt < MIN_TIME_STEP) continue;
            
            // Safely compute jerks
            if (std::isfinite(trajectory[i+1].acceleration) && 
                std::isfinite(trajectory[i].acceleration) &&
                std::isfinite(trajectory[i+1].angular_acceleration) && 
                std::isfinite(trajectory[i].angular_acceleration)) {
                
                double linear_jerk = std::abs(trajectory[i+1].acceleration - 
                                            trajectory[i].acceleration) / dt;
                double angular_jerk = std::abs(trajectory[i+1].angular_acceleration - 
                                             trajectory[i].angular_acceleration) / dt;
                
                if (std::isfinite(linear_jerk) && std::isfinite(angular_jerk)) {
                    total_jerk += std::sqrt(linear_jerk * linear_jerk + 
                                          angular_jerk * angular_jerk);
                    valid_samples++;
                }
            }
        }
        
        if (valid_samples == 0) return 1.0;
        return 1.0 / (1.0 + total_jerk / valid_samples);
        
    } catch (const std::exception& e) {
        std::cerr << "Error in computeTrajectorySmoothness: " << e.what() << std::endl;
        return 1.0;
    }
}




double PathProfiler::computeTrackingDifficulty(
    const std::vector<TrajectoryPoint>& trajectory) noexcept {
    
    if (trajectory.size() < 2) return 0.0;
    
    try {
        double total_difficulty = 0.0;
        size_t valid_samples = 0;
        
        for (size_t i = 1; i < trajectory.size(); ++i) {
            double dt = trajectory[i].time - trajectory[i-1].time;
            if (dt < MIN_TIME_STEP) continue;
            
            // Safely compute velocity changes
            if (std::isfinite(trajectory[i].velocity) && 
                std::isfinite(trajectory[i-1].velocity) &&
                std::isfinite(trajectory[i].angular_velocity) && 
                std::isfinite(trajectory[i-1].angular_velocity)) {
                
                double vel_change = std::abs(trajectory[i].velocity - 
                                           trajectory[i-1].velocity) / dt;
                double ang_vel_change = std::abs(trajectory[i].angular_velocity - 
                                               trajectory[i-1].angular_velocity) / dt;
                
                // Safely compute constraint factors
                double vel_constraint_factor = trajectory[i-1].velocity != 0.0 ? 
                    std::abs(trajectory[i].velocity / trajectory[i-1].velocity) : 1.0;
                    
                double ang_vel_constraint_factor = trajectory[i-1].angular_velocity != 0.0 ? 
                    std::abs(trajectory[i].angular_velocity / trajectory[i-1].angular_velocity) : 1.0;
                
                if (std::isfinite(vel_change) && std::isfinite(ang_vel_change) &&
                    std::isfinite(vel_constraint_factor) && 
                    std::isfinite(ang_vel_constraint_factor)) {
                    
                    double difficulty = std::sqrt(vel_change * vel_change + 
                                                ang_vel_change * ang_vel_change) +
                                      0.5 * (vel_constraint_factor + ang_vel_constraint_factor);
                    
                    if (std::isfinite(difficulty)) {
                        total_difficulty += difficulty;
                        valid_samples++;
                    }
                }
            }
        }
        
        if (valid_samples == 0) return 0.0;
        return std::min(1.0, total_difficulty / (valid_samples * 10.0));
        
    } catch (const std::exception& e) {
        std::cerr << "Error in computeTrackingDifficulty: " << e.what() << std::endl;
        return 0.0;
    }
}

PathMetrics PathProfiler::profilePath(const std::vector<Eigen::Vector3d>& path,
                                    double planning_time_ms,
                                    size_t memory_used) noexcept{
    PathMetrics metrics;
    
    // Basic path characteristics
    metrics.computation_time_ms = planning_time_ms;
    metrics.memory_usage_bytes = memory_used;
    metrics.number_of_waypoints = path.size();
    
    if (path.empty()) {
        return metrics;
    }
    
    // Path length and curvature
    metrics.total_length = computePathLength(path);
    metrics.average_curvature = computeAverageCurvature(path);
    
    // Initialize clearance metrics
    metrics.min_clearance = std::numeric_limits<double>::max();
    double total_clearance = 0.0;
    
    // Compute clearance and heading changes
    double total_heading_change = 0.0;
    metrics.max_heading_change = 0.0;
    
    for (size_t i = 0; i < path.size(); ++i) {
        // Clearance calculations
        double clearance = computeMinClearance(path[i].head<2>());
        metrics.min_clearance = std::min(metrics.min_clearance, clearance);
        total_clearance += clearance;
        
        // Heading calculations
        if (i > 0) {
            double heading_change = std::abs(computeHeadingChange(path[i-1].z(), path[i].z()));
            total_heading_change += heading_change;
            metrics.max_heading_change = std::max(metrics.max_heading_change, heading_change);
        }
    }
    
    metrics.clearance = total_clearance / path.size();
    metrics.average_heading_change = total_heading_change / (path.size() - 1);
    
    // Compute smoothness
    metrics.smoothness = computePathSmoothness(path);
    
    // Safety score (combination of clearance and curvature)
    metrics.safety_score = computeSafetyScore(path);
    
    // Goal characteristics
    if (path.size() >= 2) {
        Eigen::Vector3d goal = path.back();
        Eigen::Vector3d pre_goal = path[path.size() - 2];
        double goal_heading = std::atan2(goal.y() - pre_goal.y(), goal.x() - pre_goal.x());
        metrics.goal_alignment_error = std::abs(computeHeadingChange(goal_heading, goal.z()));
        metrics.goal_position_error = (goal.head<2>() - path.back().head<2>()).norm();
    }
    
    // Compute comfort metrics
    metrics.max_acceleration = 0.0;
    metrics.max_jerk = 0.0;
    if (path.size() >= 3) {
        for (size_t i = 1; i < path.size() - 1; ++i) {
            double dt = 0.1; // Assuming constant time step
            Eigen::Vector2d v1 = (path[i].head<2>() - path[i-1].head<2>()) / dt;
            Eigen::Vector2d v2 = (path[i+1].head<2>() - path[i].head<2>()) / dt;
            Eigen::Vector2d acc = (v2 - v1) / dt;
            
            double acceleration = acc.norm();
            metrics.max_acceleration = std::max(metrics.max_acceleration, acceleration);
            
            if (i > 1) {
                Eigen::Vector2d prev_acc = (v1 - (path[i-1].head<2>() - path[i-2].head<2>()) / dt) / dt;
                double jerk = (acc - prev_acc).norm() / dt;
                metrics.max_jerk = std::max(metrics.max_jerk, jerk);
            }
        }
    }
    
    // Compute comfort score based on acceleration and jerk
    double acc_score = 1.0 / (1.0 + metrics.max_acceleration);
    double jerk_score = 1.0 / (1.0 + metrics.max_jerk);
    metrics.comfort_score = 0.6 * acc_score + 0.4 * jerk_score;
    
    // Compute energy score based on path length and smoothness
    metrics.energy_score = 0.7 * (1.0 / (1.0 + metrics.total_length / 10.0)) + 
                          0.3 * metrics.smoothness;
    
    return metrics;
}

TrajectoryMetrics PathProfiler::profileTrajectory(
    const std::vector<TrajectoryPoint>& trajectory,
    double max_vel,
    double max_acc,
    double max_ang_vel,
    double max_ang_acc) noexcept {

    TrajectoryMetrics metrics;

    // Validate inputs
    if (trajectory.empty() || max_vel <= 0 || max_acc <= 0 || max_ang_vel <= 0 || max_ang_acc <= 0) {
        std::cerr << "Invalid inputs in profileTrajectory." << std::endl;
        return metrics;
    }

    try {
        metrics.total_duration = trajectory.back().time - trajectory.front().time;

        double total_vel = 0.0;
        double total_acc = 0.0;
        double total_ang_vel = 0.0;
        double total_ang_acc = 0.0;
        
        metrics.max_velocity = metrics.max_acceleration = metrics.max_angular_velocity = metrics.max_angular_acceleration = 0.0;

        metrics.velocity_constraints_met = metrics.acceleration_constraints_met = metrics.angular_constraints_met = true;

        double prev_time = trajectory.front().time;

        for (size_t i = 0; i < trajectory.size(); ++i) {
            const auto& point = trajectory[i];
            
            // Debug: log current trajectory index, time, and velocity values
            std::cerr << "[DEBUG] Processing trajectory point " << i
                      << " | time: " << point.time
                      << " | prev_time: " << prev_time
                      << " | velocity: " << point.velocity
                      << " | acceleration: " << point.acceleration
                      << " | angular_velocity: " << point.angular_velocity
                      << " | angular_acceleration: " << point.angular_acceleration << std::endl;

            // Check time consistency and validity
            if (point.time <= prev_time || !std::isfinite(point.time)) {
                std::cerr << "[ERROR] Invalid or non-incremental timestamp at point " << i
                          << " | time: " << point.time
                          << " | prev_time: " << prev_time << std::endl;
                prev_time = point.time;
                continue;
            }

            double dt = point.time - prev_time;
            if (dt < MIN_TIME_STEP) {
                std::cerr << "[INFO] Skipping point due to insufficient dt at index " << i << std::endl;
                prev_time = point.time;
                continue;
            }

            // Velocity handling
            if (std::isfinite(point.velocity)) {
                total_vel += std::abs(point.velocity);
                metrics.max_velocity = std::max(metrics.max_velocity, std::abs(point.velocity));
                metrics.velocity_constraints_met &= (std::abs(point.velocity) <= max_vel);
            } else {
                std::cerr << "[WARNING] Non-finite velocity at point " << i << std::endl;
            }

            // Acceleration handling
            if (std::isfinite(point.acceleration)) {
                total_acc += std::abs(point.acceleration);
                metrics.max_acceleration = std::max(metrics.max_acceleration, std::abs(point.acceleration));
                metrics.acceleration_constraints_met &= (std::abs(point.acceleration) <= max_acc);
            } else {
                std::cerr << "[WARNING] Non-finite acceleration at point " << i << std::endl;
            }

            // Angular velocity handling
            if (std::isfinite(point.angular_velocity)) {
                total_ang_vel += std::abs(point.angular_velocity);
                metrics.max_angular_velocity = std::max(metrics.max_angular_velocity, std::abs(point.angular_velocity));
            } else {
                std::cerr << "[WARNING] Non-finite angular velocity at point " << i << std::endl;
            }

            // Angular acceleration handling
            if (std::isfinite(point.angular_acceleration)) {
                total_ang_acc += std::abs(point.angular_acceleration);
                metrics.max_angular_acceleration = std::max(metrics.max_angular_acceleration, std::abs(point.angular_acceleration));
            } else {
                std::cerr << "[WARNING] Non-finite angular acceleration at point " << i << std::endl;
            }

            // Angular constraints
            metrics.angular_constraints_met &= 
                (std::isfinite(point.angular_velocity) && std::abs(point.angular_velocity) <= max_ang_vel) &&
                (std::isfinite(point.angular_acceleration) && std::abs(point.angular_acceleration) <= max_ang_acc);

            prev_time = point.time;
        }

        size_t valid_points = trajectory.size();
        if (valid_points > 0) {
            metrics.average_velocity = total_vel / valid_points;
            metrics.average_acceleration = total_acc / valid_points;
            metrics.average_angular_velocity = total_ang_vel / valid_points;
            metrics.average_angular_acceleration = total_ang_acc / valid_points;
        }

        // Compute smoothness and tracking difficulty safely
        metrics.smoothness = computeTrajectorySmoothness(trajectory);
        metrics.tracking_difficulty = computeTrackingDifficulty(trajectory);

    } catch (const std::exception& e) {
        std::cerr << "Error in profileTrajectory: " << e.what() << std::endl;
        return TrajectoryMetrics();
    }

    return metrics;
}




void PathProfiler::plotPerformanceMetrics(
    const std::vector<TrajectoryPoint>& trajectory,
    const std::string& output_dir) noexcept {
    
    try {
        // 1. Early validation with proper error handling
        if (trajectory.empty()) {
            std::cerr << "ERROR: Empty trajectory provided" << std::endl;
            return;
        }

        // 2. Pre-validate trajectory data before reserving space
        size_t valid_points = 0;
        for (const auto& point : trajectory) {
            if (point.time >= 0 &&  // Basic sanity check for time
                std::isfinite(point.time) && 
                std::isfinite(point.velocity) && 
                std::isfinite(point.acceleration) &&
                std::isfinite(point.angular_velocity) && 
                std::isfinite(point.angular_acceleration)) {
                valid_points++;
            }
        }

        if (valid_points == 0) {
            std::cerr << "ERROR: No valid trajectory points found" << std::endl;
            return;
        }

        // 3. Safe vector initialization with try-catch
        std::vector<double> times, velocities, accelerations, ang_velocities, ang_accelerations;
        
        try {
            // Only reserve if we have a reasonable number of points
            if (valid_points > 1000000) {
                std::cerr << "ERROR: Too many points (" << valid_points << "). Possible corrupt data." << std::endl;
                return;
            }
            
            times.reserve(valid_points);
            velocities.reserve(valid_points);
            accelerations.reserve(valid_points);
            ang_velocities.reserve(valid_points);
            ang_accelerations.reserve(valid_points);
        } catch (const std::bad_alloc& e) {
            std::cerr << "ERROR: Memory allocation failed: " << e.what() << std::endl;
            return;
        }

        // 4. Safely populate vectors with validated data
        double t0 = trajectory.front().time;
        for (const auto& point : trajectory) {
            if (point.time >= 0 &&
                std::isfinite(point.time) && 
                std::isfinite(point.velocity) && 
                std::isfinite(point.acceleration) &&
                std::isfinite(point.angular_velocity) && 
                std::isfinite(point.angular_acceleration)) {
                
                try {
                    times.push_back(point.time - t0);
                    velocities.push_back(point.velocity);
                    accelerations.push_back(point.acceleration);
                    ang_velocities.push_back(point.angular_velocity);
                    ang_accelerations.push_back(point.angular_acceleration);
                } catch (const std::exception& e) {
                    std::cerr << "ERROR: Failed to add point to vectors: " << e.what() << std::endl;
                    return;
                }
            }
        }

        // 5. Verify we have enough data to plot
        if (times.size() < 2) {
            std::cerr << "ERROR: Insufficient valid points for plotting" << std::endl;
            return;
        }

        // 6. Create output directory with error handling
        try {
            std::filesystem::create_directories(output_dir);
        } catch (const std::exception& e) {
            std::cerr << "ERROR: Failed to create output directory: " << e.what() << std::endl;
            return;
        }

        // 7. Plot each metric with proper error handling
        try {
            // Velocity profile
            plt::figure_size(10, 6);
            plt::plot(times, velocities);
            plt::title("Velocity Profile");
            plt::xlabel("Time (s)");
            plt::ylabel("Velocity (m/s)");
            plt::grid(true);
            plt::save(output_dir + "/velocity_profile.png");
            plt::close();

            // Acceleration profile
            plt::figure_size(10, 6);
            plt::plot(times, accelerations);
            plt::title("Acceleration Profile");
            plt::xlabel("Time (s)");
            plt::ylabel("Acceleration (m/s²)");
            plt::grid(true);
            plt::save(output_dir + "/acceleration_profile.png");
            plt::close();

            // Angular velocity profile
            plt::figure_size(10, 6);
            plt::plot(times, ang_velocities);
            plt::title("Angular Velocity Profile");
            plt::xlabel("Time (s)");
            plt::ylabel("Angular Velocity (rad/s)");
            plt::grid(true);
            plt::save(output_dir + "/angular_velocity_profile.png");
            plt::close();

            // Angular acceleration profile
            plt::figure_size(10, 6);
            plt::plot(times, ang_accelerations);
            plt::title("Angular Acceleration Profile");
            plt::xlabel("Time (s)");
            plt::ylabel("Angular Acceleration (rad/s²)");
            plt::grid(true);
            plt::save(output_dir + "/angular_acceleration_profile.png");
            plt::close();

        } catch (const std::exception& e) {
            std::cerr << "ERROR: Failed during plotting: " << e.what() << std::endl;
            return;
        }

    } catch (const std::exception& e) {
        std::cerr << "ERROR: Unexpected error in plotPerformanceMetrics: " << e.what() << std::endl;
    }
}

// New safe plotting function
void PathProfiler::plotTrajectory(
    const std::vector<TrajectoryPoint>& trajectory,
    const std::string& output_dir) const noexcept {
    
    try {
        if (trajectory.empty()) {
            std::cerr << "[ERROR] Empty trajectory, nothing to plot\n";
            return;
        }

        std::filesystem::create_directories(output_dir);

        const size_t MAX_PLOT_POINTS = 1000;
        size_t step = trajectory.size() > MAX_PLOT_POINTS ? trajectory.size() / MAX_PLOT_POINTS : 1;

        std::vector<double> times, velocities, accelerations, x_pos, y_pos;

        try {
            times.reserve(MAX_PLOT_POINTS);
            velocities.reserve(MAX_PLOT_POINTS);
            accelerations.reserve(MAX_PLOT_POINTS);
            x_pos.reserve(MAX_PLOT_POINTS);
            y_pos.reserve(MAX_PLOT_POINTS);
        } catch (const std::bad_alloc& e) {
            std::cerr << "[ERROR] Memory allocation failed for plotting vectors: " << e.what() << std::endl;
            return;
        }

        double start_time = trajectory[0].time;
        for (size_t i = 0; i < trajectory.size(); i += step) {
            const auto& point = trajectory[i];
            times.push_back(point.time - start_time);
            velocities.push_back(point.velocity);
            accelerations.push_back(point.acceleration);
            x_pos.push_back(point.position.x());
            y_pos.push_back(point.position.y());
        }

        plt::figure_size(10, 10);
        plt::plot(x_pos, y_pos, "b-");
        plt::title("Robot Trajectory");
        plt::xlabel("X Position (m)");
        plt::ylabel("Y Position (m)");
        plt::axis("equal");
        plt::grid(true);
        plt::save(output_dir + "/trajectory_path.png");
        plt::close();

        plt::figure_size(10, 6);
        plt::plot(times, velocities);
        plt::title("Velocity Profile");
        plt::xlabel("Time (s)");
        plt::ylabel("Velocity (m/s)");
        plt::grid(true);
        plt::save(output_dir + "/velocity_profile.png");
        plt::close();

        plt::figure_size(10, 6);
        plt::plot(times, accelerations);
        plt::title("Acceleration Profile");
        plt::xlabel("Time (s)");
        plt::ylabel("Acceleration (m/s²)");
        plt::grid(true);
        plt::save(output_dir + "/acceleration_profile.png");
        plt::close();

    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception in plotTrajectory: " << e.what() << std::endl;
    }
}

void PathProfiler::visualizeMetrics(
    PathMetrics& path_metrics,
    const TrajectoryMetrics& traj_metrics,
    const std::string& output_dir) noexcept {
    
    try {
        // Create directory if it doesn't exist
        std::filesystem::create_directories(output_dir);

        // Pre-allocate vectors
        std::vector<double> quality_metrics, performance_metrics;
        std::vector<std::string> quality_labels, performance_labels;

        // Reserve space
        quality_metrics.reserve(5);
        quality_labels.reserve(5);
        performance_metrics.reserve(5);
        performance_labels.reserve(5);

        // Path quality metrics
        quality_metrics = {
            path_metrics.smoothness,
            path_metrics.safety_score,
            path_metrics.clearance / (5.0 * robot_radius_),
            path_metrics.comfort_score,
            path_metrics.energy_score
        };
        
        quality_labels = {
            "Smoothness",
            "Safety",
            "Clearance",
            "Comfort",
            "Energy"
        };

        // Performance metrics
        performance_metrics = {
            safeDivide(traj_metrics.average_velocity, traj_metrics.max_velocity),
            safeDivide(traj_metrics.average_acceleration, traj_metrics.max_acceleration),
            safeDivide(traj_metrics.average_angular_velocity, traj_metrics.max_angular_velocity),
            traj_metrics.smoothness,
            1.0 - traj_metrics.tracking_difficulty
        };
        
        performance_labels = {
            "Velocity Util",
            "Accel Util",
            "Angular Vel Util",
            "Smoothness",
            "Tracking Ease"
        };

        // Generate plots
        std::vector<double> x(quality_metrics.size());
        std::iota(x.begin(), x.end(), 0);

        // Plot quality metrics
        plt::figure_size(12, 6);
        plt::bar(x, quality_metrics);
        plt::xticks(x, quality_labels);
        plt::ylim(0.0, 1.0);
        plt::title("Path Quality Metrics");
        plt::grid(true);
        plt::save(output_dir + "/path_quality_metrics.png");
        plt::close();

        // Plot performance metrics
        plt::figure_size(12, 6);
        plt::bar(x, performance_metrics);
        plt::xticks(x, performance_labels);
        plt::ylim(0.0, 1.0);
        plt::title("Trajectory Performance Metrics");
        plt::grid(true);
        plt::save(output_dir + "/trajectory_performance_metrics.png");
        plt::close();

    } catch (const std::exception& e) {
        std::cerr << "Error in visualizeMetrics: " << e.what() << std::endl;
    }
}

double PathProfiler::safeDivide(double numerator, double denominator) const noexcept {
    if (std::abs(denominator) < 1e-10 || !std::isfinite(denominator) || !std::isfinite(numerator)) {
        return 0.0;
    }
    return numerator / denominator;
}