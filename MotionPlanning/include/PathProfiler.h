#ifndef PATH_PROFILER_H
#define PATH_PROFILER_H

#include <vector>
#include <string>
#include <filesystem>
#include <Eigen/Dense>
#include "common_defs.h"
#include "OccupancyGrid.h"

class PathProfiler {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // Constructor with validity checks
    PathProfiler(const OccupancyGrid& grid, double robot_radius);
    
    // Main profiling methods with additional parameter validation
    PathMetrics profilePath(const std::vector<Eigen::Vector3d>& path, 
                          double planning_time_ms = 0.0,
                          size_t memory_used = 0) noexcept;
    
    TrajectoryMetrics profileTrajectory(const std::vector<TrajectoryPoint>& trajectory,
                                      double max_vel,
                                      double max_acc,
                                      double max_ang_vel,
                                      double max_ang_acc) noexcept;
    
    void visualizeMetrics(PathMetrics& path_metrics,
                         const TrajectoryMetrics& traj_metrics,
                         const std::string& output_dir = ".") noexcept;
    void plotTrajectory(const std::vector<TrajectoryPoint>& trajectory,
                       const std::string& output_dir) const noexcept;

private:
    const OccupancyGrid& grid_;
    double robot_radius_;
    
    // Constants for metric calculations
    static constexpr double MIN_SEGMENT_LENGTH = 1e-6;
    static constexpr double MIN_TIME_STEP = 1e-6;
    static constexpr double DEFAULT_TIME_STEP = 0.1;
    static constexpr double MAX_HEADING_DIFF = M_PI;
    
    // Helper methods with bounds checking and error handling
    double computePathLength(const std::vector<Eigen::Vector3d>& path) noexcept;
    double computeAverageCurvature(const std::vector<Eigen::Vector3d>& path) noexcept;
    double computePathSmoothness(const std::vector<Eigen::Vector3d>& path) noexcept;
    double computeSafetyScore(const std::vector<Eigen::Vector3d>& path) noexcept;
    double computeMinClearance(const Eigen::Vector2d& position) noexcept;
    double computeHeadingChange(double h1, double h2) noexcept;
    double computeTrajectorySmoothness(const std::vector<TrajectoryPoint>& trajectory) noexcept;
    double computeTrackingDifficulty(const std::vector<TrajectoryPoint>& trajectory) noexcept;
    
    // Validation methods
    bool validateTrajectoryPoint(const TrajectoryPoint& point) const noexcept;
    bool validatePath(const std::vector<Eigen::Vector3d>& path) const noexcept;
    bool validateTrajectory(const std::vector<TrajectoryPoint>& trajectory) const noexcept;
    
    // Safe numerical calculations
    double safeNorm(const Eigen::Vector2d& vec) const noexcept;
    double safeDivide(double numerator, double denominator) const noexcept;
    double boundAngle(double angle) const noexcept;
    
    // Internal plotting helpers
    void plotPerformanceMetrics(const std::vector<TrajectoryPoint>& trajectory,
                              const std::string& output_dir) noexcept;
    void ensureValidOutputDir(const std::string& output_dir) noexcept;
    
    // Prevent copying and assignment
    PathProfiler(const PathProfiler&) = delete;
    PathProfiler& operator=(const PathProfiler&) = delete;
};

#endif // PATH_PROFILER_H