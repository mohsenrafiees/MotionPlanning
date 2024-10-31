#ifndef COMMON_DEFS_H
#define COMMON_DEFS_H

#include <memory>
#include <vector>
#include <Eigen/Dense>
#include "BasePlanner.h"
#include "OccupancyGrid.h"
#include "TrajectoryPlanner.h"

// Enum for planner types
enum class PlannerType {
    AStar,
    ThetaStar,
    SBPL,
    RRTStar,
    CHOMPPlanner,
    STOMPPlanner
};

// Forward declarations of planner classes
class AStarPlanner;
class ThetaStarPlanner;
class SBPLPlanner;

// Common metrics for path quality
struct PathMetrics {
    double total_length;                    // Total path length in meters
    double average_curvature;               // Average path curvature
    double max_curvature;                   // Maximum path curvature
    double smoothness;                      // Path smoothness metric (0-1)
    double clearance;                       // Average distance to obstacles
    double min_clearance;                   // Minimum distance to obstacles
    double computation_time_ms;             // Planning computation time
    size_t memory_usage_bytes;              // Memory usage during planning
    size_t number_of_waypoints;             // Number of path points
    double safety_score;                    // Overall safety rating (0-1)
    double min_obstacle_distance;           // Minimum distance to any obstacle
    double max_heading_change;              // Maximum heading change between segments
    double average_heading_change;          // Average heading change
    double comfort_score;                   // Overall comfort rating (0-1)
    double max_acceleration;                // Maximum acceleration in path
    double max_jerk;                        // Maximum jerk in path
    double energy_score;                    // Estimated energy efficiency (0-1)
    double goal_alignment_error;            // Final heading alignment error
    double goal_position_error;             // Distance to exact goal position
    double path_score;                      // Path Quality Final Score:
};

// Common metrics for trajectory quality
struct TrajectoryMetrics {
    double total_duration;                  // Total trajectory duration
    double average_velocity;                // Average velocity
    double max_velocity;                    // Maximum velocity reached
    double average_acceleration;            // Average acceleration
    double max_acceleration;                // Maximum acceleration
    double average_angular_velocity;        // Average angular velocity
    double max_angular_velocity;            // Maximum angular velocity
    double average_angular_acceleration;    // Average angular acceleration
    double max_angular_acceleration;        // Maximum angular acceleration
    bool velocity_constraints_met;          // Whether velocity limits were respected
    bool acceleration_constraints_met;      // Whether acceleration limits were respected
    bool angular_constraints_met;           // Whether angular motion limits were respected
    double tracking_difficulty;             // Estimated difficulty of trajectory tracking (0-1)
    double smoothness;                      // Trajectory smoothness metric (0-1)
};

// Common result structure for planners
struct PlannerResults {
    std::vector<Eigen::Vector3d> path;
    std::vector<TrajectoryPoint> trajectory;
    PathMetrics path_metrics;
    TrajectoryMetrics traj_metrics;

    PlannerResults() = default;
    
    PlannerResults(const std::vector<Eigen::Vector3d>& p,
                  const std::vector<TrajectoryPoint>& t,
                  const PathMetrics& pm,
                  const TrajectoryMetrics& tm)
        : path(p), trajectory(t), path_metrics(pm), traj_metrics(tm) {}
};

#endif // COMMON_DEFS_H