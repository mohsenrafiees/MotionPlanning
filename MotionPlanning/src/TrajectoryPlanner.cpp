#include "TrajectoryPlanner.h"
#include "OccupancyGrid.h"
#include <iostream>
#include <limits>
#include <cmath>

TrajectoryPlanner::TrajectoryPlanner(
    double max_vel, double max_acc, double max_angular_vel,
    double max_angular_acc, double min_vel,
    double min_turning_radius, double caution_threshold,
    double merge_threshold)
    : max_vel_(std::max(0.1, max_vel)),
      max_acc_(std::max(0.1, max_acc)),
      max_angular_vel_(std::max(0.1, max_angular_vel)),
      max_angular_acc_(std::max(0.1, max_angular_acc)),
      min_vel_(std::max(0.05, min_vel)),
      min_turning_radius_(std::max(0.1, min_turning_radius)),
      caution_threshold_(std::max(0.1, caution_threshold)),
      merge_threshold_(std::max(0.01, merge_threshold)) {
    
    std::cout << "Trajectory Planner initialized with validated parameters" << std::endl;
}

bool TrajectoryPlanner::validateInputs(
    const std::vector<Eigen::Vector3d>& path,
    double robot_radius,
    double time_step) const noexcept {
    
    if (path.empty()) {
        std::cerr << "Error: Empty path provided" << std::endl;
        return false;
    }

    if (path.size() > MAX_TRAJECTORY_POINTS) {
        std::cerr << "Error: Path too long (" << path.size() << " points)" << std::endl;
        return false;
    }

    if (robot_radius <= 0.0) {
        std::cerr << "Error: Invalid robot radius" << std::endl;
        return false;
    }

    if (time_step < MIN_TIME_STEP) {
        std::cerr << "Error: Time step too small" << std::endl;
        return false;
    }

    for (const auto& point : path) {
        if (!point.allFinite()) {
            std::cerr << "Error: Invalid path point detected" << std::endl;
            return false;
        }
    }

    return true;
}
void TrajectoryPlanner::generateSmoothTrajectory(
    const OccupancyGrid& occupancy_grid,
    const std::vector<Eigen::Vector3d>& path,
    std::vector<TrajectoryPoint>& trajectory,
    double robot_radius,
    double time_step) {
    
    trajectory.clear();
    if (path.size() < 2) return;

    // Calculate full path length for final deceleration planning
    double total_path_length = 0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        total_path_length += (path[i+1].head<2>() - path[i].head<2>()).norm();
    }
    std::cout << "Total path length: " << total_path_length << "m" << std::endl;

    // Backward pass first to determine velocity limits
    struct SegmentInfo {
        double start_velocity;
        double end_velocity;
        double length;
        double clearance;
    };
    
    std::vector<SegmentInfo> segments;
    segments.reserve(path.size() - 1);

    // Forward pass to gather segment information
    for (size_t i = 0; i < path.size() - 1; ++i) {
        SegmentInfo segment;
        segment.length = (path[i+1].head<2>() - path[i].head<2>()).norm();
        
        // Get base velocity limit from obstacles
        double base_limit = calculateVelocityLimit(
            occupancy_grid,
            path[i].head<2>(),
            path[i+1].head<2>(),
            robot_radius,
            0.0
        );
        
        // Store clearance for debugging
        segment.clearance = occupancy_grid.getDistanceToNearestObstacle(path[i].head<2>());
        
        // Initialize velocities (will be updated in backward pass)
        segment.start_velocity = base_limit;
        segment.end_velocity = base_limit;
        
        segments.push_back(segment);
        
        std::cout << "Segment " << i << ": length=" << segment.length 
                  << "m, base_limit=" << base_limit 
                  << "m/s, clearance=" << segment.clearance << "m" << std::endl;
    }

    // Backward pass to propagate velocity constraints
    segments.back().end_velocity = 0.0;  // Final stop condition
    for (int i = segments.size() - 1; i >= 0; --i) {
        // Calculate max velocity that allows stopping at end
        if (i < static_cast<int>(segments.size()) - 1) {
            double next_vel = segments[i + 1].start_velocity;
            double decel_dist = (segments[i].end_velocity * segments[i].end_velocity - 
                               next_vel * next_vel) / (2.0 * max_acc_);
                               
            if (decel_dist > segments[i].length) {
                // Need to start slowing down in this segment
                segments[i].end_velocity = std::sqrt(
                    2.0 * max_acc_ * segments[i].length + 
                    next_vel * next_vel
                );
            }
        }
        
        // Propagate backwards
        segments[i].start_velocity = std::min(
            segments[i].start_velocity,
            std::sqrt(
                2.0 * max_acc_ * segments[i].length + 
                segments[i].end_velocity * segments[i].end_velocity
            )
        );
    }

    // Generate trajectory points
    double current_time = time_step;  // Start at first time step to avoid t=0 issues
    double current_vel = min_vel_;
    
    TrajectoryPoint start_point;
    start_point.position = path[0];
    start_point.velocity = current_vel;
    start_point.acceleration = 0.0;
    start_point.time = 0.0;
    trajectory.push_back(start_point);

    size_t current_segment = 0;
    double distance_in_segment = 0.0;

    while (current_segment < segments.size()) {
        const auto& segment = segments[current_segment];
        
        // Calculate target velocity based on position in segment
        double segment_progress = distance_in_segment / segment.length;
        double target_vel = segment.start_velocity + 
            (segment.end_velocity - segment.start_velocity) * segment_progress;

        // Calculate acceleration
        double vel_diff = target_vel - current_vel;
        double acceleration = std::clamp(
            vel_diff / time_step,
            -max_acc_,
            max_acc_
        );

        // Update velocity
        current_vel = std::clamp(
            current_vel + acceleration * time_step,
            min_vel_,
            max_vel_  // Allow full velocity range
        );

        // Calculate distance covered in this step
        double step_distance = current_vel * time_step;
        distance_in_segment += step_distance;

        // Create trajectory point
        TrajectoryPoint point;
        point.time = current_time;
        point.velocity = current_vel;
        point.acceleration = acceleration;
        
        // Update position
        Eigen::Vector2d direction = (path[current_segment + 1].head<2>() - 
                                   path[current_segment].head<2>()).normalized();
        point.position = trajectory.back().position;
        point.position.head<2>() += direction * step_distance;
        point.heading = std::atan2(direction.y(), direction.x());

        trajectory.push_back(point);
        current_time += time_step;

        // Check if we need to move to next segment
        if (distance_in_segment >= segment.length) {
            distance_in_segment = 0.0;
            current_segment++;
        }

        // Debug output every 20 points
        if (trajectory.size() % 20 == 0) {
            std::cout << "t=" << current_time 
                      << " v=" << current_vel 
                      << " a=" << acceleration 
                      << " segment=" << current_segment 
                      << " progress=" << segment_progress << std::endl;
        }
    }

    // Add final stop point if needed
    if (trajectory.back().velocity > 0.01) {
        TrajectoryPoint stop_point = trajectory.back();
        stop_point.velocity = 0.0;
        stop_point.acceleration = 0.0;
        stop_point.time += time_step;
        trajectory.push_back(stop_point);
    }

    std::cout << "Generated trajectory with " << trajectory.size() 
              << " points over " << current_time << " seconds" << std::endl;
}

// Helper function for smooth acceleration profile
double TrajectoryPlanner::smoothstep(double t) const noexcept {
    t = std::clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);  // Cubic Hermite interpolation
}
double TrajectoryPlanner::calculateVelocityLimit(
    const OccupancyGrid& grid,
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& end,
    double robot_radius,
    double heading_change) noexcept {
    
    try {
        double velocity_limit = max_vel_;
        
        // Check clearance along path
        Eigen::Vector2d direction = (end - start).normalized();
        double distance = (end - start).norm();
        double min_clearance = std::numeric_limits<double>::max();
        
        for (double d = 0; d <= distance; d += grid.resolution_m) {
            Eigen::Vector2d point = start + d * direction;
            double clearance = grid.getDistanceToNearestObstacle(point);
            min_clearance = std::min(min_clearance, clearance);
            
            if (clearance < caution_threshold_) {
                double scale = std::clamp(
                    (clearance - robot_radius) / (caution_threshold_ - robot_radius),
                    0.3,  // Minimum 30% of max velocity
                    1.0
                );
                velocity_limit = std::min(velocity_limit, max_vel_ * scale);
            }
        }
        
        std::cout << "Velocity limit calculation - distance: " << distance 
                  << "m, min clearance: " << min_clearance 
                  << "m, limit: " << velocity_limit << " m/s" << std::endl;
        
        return std::max(min_vel_, velocity_limit);
        
    } catch (const std::exception& e) {
        std::cerr << "Error in velocity limit calculation: " << e.what() << std::endl;
        return min_vel_;
    }
}
double TrajectoryPlanner::interpolateAngle(double start, double end, double t) noexcept{
        double diff = normalizeAngle(end - start);
        return normalizeAngle(start + diff * t);
    }
bool TrajectoryPlanner::generateTrajectorySegment(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    double velocity_limit,
    double segment_length,
    double heading_change,
    double time_step,
    std::vector<TrajectoryPoint>& segment) noexcept {
    
    try {
        segment.clear();
        
        // Time allocation based on segment properties
        double linear_time = segment_length / velocity_limit;
        double angular_time = std::abs(heading_change) / max_angular_vel_;
        double segment_time = std::max(linear_time, angular_time);
        
        size_t num_points = static_cast<size_t>(segment_time / time_step) + 1;
        if (num_points > MAX_TRAJECTORY_POINTS) {
            std::cerr << "Warning: Segment would generate too many points" << std::endl;
            return false;
        }
        
        segment.reserve(num_points);
        
        double current_time = 0.0;
        while (current_time <= segment_time) {
            TrajectoryPoint point;
            double t = current_time / segment_time;  // Normalized time
            
            // Use quintic polynomial for smooth acceleration
            double h = quinticPolynomial(t);
            double dh = quinticPolynomialDerivative(t) / segment_time;
            double ddh = quinticPolynomialSecondDerivative(t) / (segment_time * segment_time);
            
            // Position interpolation
            point.position = start + h * (end - start);
            
            // Velocity and acceleration
            point.velocity = dh * segment_length;
            point.acceleration = ddh * segment_length;
            
            // Heading interpolation
            point.heading = normalizeAngle(start.z() + h * heading_change);
            point.heading_change = heading_change;
            
            // Angular motion
            point.angular_velocity = dh * heading_change;
            point.angular_acceleration = ddh * heading_change;
            
            // Store velocity limit for profiling
            point.curvature_velocity_limit = velocity_limit;
            
            // Time tracking
            point.time = current_time;
            
            // Cautious segment flagging
            point.in_cautious_segment = velocity_limit < max_vel_;
            
            segment.push_back(point);
            current_time += time_step;
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error in generateTrajectorySegment: " << e.what() << std::endl;
        segment.clear();
        return false;
    }
}

bool TrajectoryPlanner::mergeSmoothly(
    std::vector<TrajectoryPoint>& main_trajectory,
    const std::vector<TrajectoryPoint>& segment) noexcept {
    
    try {
        if (segment.empty()) return true;
        if (main_trajectory.empty()) {
            main_trajectory = segment;
            return true;
        }
        
        // Time offset for the new segment
        double time_offset = main_trajectory.back().time;
        
        // Blend region
        size_t blend_start = main_trajectory.size() > 5 ? main_trajectory.size() - 5 : 0;
        
        // Check if merge would exceed size limit
        if (main_trajectory.size() + segment.size() - 5 > MAX_TRAJECTORY_POINTS) {
            std::cerr << "Warning: Merge would exceed maximum trajectory size" << std::endl;
            return false;
        }
        
        for (size_t i = 0; i < segment.size(); ++i) {
            TrajectoryPoint point = segment[i];
            point.time += time_offset;
            
            // Apply blending in the overlap region
            if (i < 5 && blend_start < main_trajectory.size()) {
                double blend_factor = static_cast<double>(i) / 5.0;
                
                auto& blend_point = main_trajectory[blend_start + i];
                point.position = blend_point.position * (1.0 - blend_factor) + 
                               point.position * blend_factor;
                point.velocity = blend_point.velocity * (1.0 - blend_factor) + 
                               point.velocity * blend_factor;
                point.acceleration = blend_point.acceleration * (1.0 - blend_factor) + 
                               point.acceleration * blend_factor;
                point.heading = normalizeAngle(
                    blend_point.heading * (1.0 - blend_factor) + 
                    point.heading * blend_factor
                );
                point.angular_velocity = blend_point.angular_velocity * (1.0 - blend_factor) + 
                                      point.angular_velocity * blend_factor;
                point.angular_acceleration = blend_point.angular_acceleration * (1.0 - blend_factor) + 
                                         point.angular_acceleration * blend_factor;
            }
            
            if (i >= 5) {
                main_trajectory.push_back(point);
            } else {
                main_trajectory[blend_start + i] = point;
            }
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error in mergeSmoothly: " << e.what() << std::endl;
        return false;
    }
}

void TrajectoryPlanner::smoothTrajectory(
    std::vector<TrajectoryPoint>& trajectory,
    double time_step) noexcept {
    
    try {
        if (trajectory.size() < 3) return;
        
        const int window_size = 5;
        std::vector<TrajectoryPoint> smoothed = trajectory;
        
        for (size_t i = window_size; i < trajectory.size() - window_size; ++i) {
            Eigen::Vector3d avg_pos = Eigen::Vector3d::Zero();
            double avg_vel = 0.0, avg_acc = 0.0;
            double avg_ang_vel = 0.0, avg_ang_acc = 0.0;
            double total_weight = 0.0;
            
            // Apply Gaussian-weighted moving average
            for (int j = -window_size; j <= window_size; ++j) {
                double weight = std::exp(-j*j / (2.0 * window_size * window_size));
                total_weight += weight;
                
                avg_pos += weight * trajectory[i+j].position;
                avg_vel += weight * trajectory[i+j].velocity;
                avg_acc += weight * trajectory[i+j].acceleration;
                avg_ang_vel += weight * trajectory[i+j].angular_velocity;
                avg_ang_acc += weight * trajectory[i+j].angular_acceleration;
            }
            
            if (total_weight > 0.0) {
                smoothed[i].position = avg_pos / total_weight;
                smoothed[i].velocity = avg_vel / total_weight;
                smoothed[i].acceleration = avg_acc / total_weight;
                smoothed[i].angular_velocity = avg_ang_vel / total_weight;
                smoothed[i].angular_acceleration = avg_ang_acc / total_weight;
            }
        }
        
        trajectory = smoothed;
        
    } catch (const std::exception& e) {
        std::cerr << "Error in smoothTrajectory: " << e.what() << std::endl;
    }
}

void TrajectoryPlanner::enforceKinematicConstraints(
    std::vector<TrajectoryPoint>& trajectory) noexcept {
    
    try {
        for (auto& point : trajectory) {
            // Enforce velocity constraints
            point.velocity = std::clamp(point.velocity, -max_vel_, max_vel_);
            point.acceleration = std::clamp(point.acceleration, -max_acc_, max_acc_);
            point.angular_velocity = std::clamp(point.angular_velocity, 
                                              -max_angular_vel_, max_angular_vel_);
            point.angular_acceleration = std::clamp(point.angular_acceleration,
                                                  -max_angular_acc_, max_angular_acc_);
            
            // Ensure all values are finite
            if (!std::isfinite(point.velocity) || 
                !std::isfinite(point.acceleration) ||
                !std::isfinite(point.angular_velocity) ||
                !std::isfinite(point.angular_acceleration)) {
                
                point.velocity = 0.0;
                point.acceleration = 0.0;
                point.angular_velocity = 0.0;
                point.angular_acceleration = 0.0;
            }
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error in enforceKinematicConstraints: " << e.what() << std::endl;
    }
}
double TrajectoryPlanner::quinticPolynomial(double t) const noexcept {
    if (t < 0.0) return 0.0;
    if (t > 1.0) return 1.0;
    return 10 * std::pow(t, 3) - 15 * std::pow(t, 4) + 6 * std::pow(t, 5);
}

double TrajectoryPlanner::quinticPolynomialDerivative(double t) const noexcept {
    if (t < 0.0 || t > 1.0) return 0.0;
    return 30 * std::pow(t, 2) - 60 * std::pow(t, 3) + 30 * std::pow(t, 4);
}

double TrajectoryPlanner::quinticPolynomialSecondDerivative(double t) const noexcept {
    if (t < 0.0 || t > 1.0) return 0.0;
    return 60 * t - 180 * std::pow(t, 2) + 120 * std::pow(t, 3);
}

double TrajectoryPlanner::normalizeAngle(double angle) const noexcept {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}