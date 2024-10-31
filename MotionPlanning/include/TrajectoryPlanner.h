#pragma once
#include <vector>
#include <Eigen/Dense>
#include <memory>
#include <stdexcept>

class OccupancyGrid;

struct TrajectoryPoint {
    double time = 0.0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    double velocity = 0.0;
    double acceleration = 0.0;
    double heading = 0.0;
    double angular_velocity = 0.0;
    double angular_acceleration = 0.0;
    double curvature_velocity_limit = 0.0;
    double heading_change = 0.0;
    bool in_cautious_segment = false;
    
    TrajectoryPoint() = default;
};

class TrajectoryPlanner {
public:
    static constexpr double MIN_SEGMENT_LENGTH = 0.01;  // 1cm
    static constexpr double MIN_TIME_STEP = 0.01;       // 10ms
    static constexpr size_t MAX_TRAJECTORY_POINTS = 10000; // Safety limit
    static constexpr double VELOCITY_THRESHOLD = 0.001;  // m/s
    static constexpr double ACCELERATION_LIMIT = 0.5;    // m/s^2
    static constexpr double DEFAULT_VELOCITY = 0.3;      // m/s

    TrajectoryPlanner(double max_vel, double max_acc, double max_angular_vel,
                     double max_angular_acc, double min_vel,
                     double min_turning_radius, double caution_threshold,
                     double merge_threshold);

    void generateSmoothTrajectory(
        const OccupancyGrid& occupancy_grid,
        const std::vector<Eigen::Vector3d>& path,
        std::vector<TrajectoryPoint>& trajectory,
        double robot_radius,
        double time_step);

private:
    double max_vel_;
    double max_acc_;
    double max_angular_vel_;
    double max_angular_acc_;
    double min_vel_;
    double min_turning_radius_;
    double caution_threshold_;
    double merge_threshold_;

    bool validateInputs(const std::vector<Eigen::Vector3d>& path, double robot_radius, 
                       double time_step) const noexcept;
    double smoothstep(double t) const noexcept;
                       
    double calculateVelocityLimit(
        const OccupancyGrid& grid,
        const Eigen::Vector2d& start,
        const Eigen::Vector2d& end,
        double robot_radius,
        double heading_change) noexcept;

    bool generateTrajectorySegment(
        const Eigen::Vector3d& start,
        const Eigen::Vector3d& end,
        double velocity_limit,
        double segment_length,
        double heading_change,
        double time_step,
        std::vector<TrajectoryPoint>& segment) noexcept;

    bool mergeSmoothly(
        std::vector<TrajectoryPoint>& main_trajectory,
        const std::vector<TrajectoryPoint>& segment) noexcept;

    void smoothTrajectory(
        std::vector<TrajectoryPoint>& trajectory,
        double time_step) noexcept;

    void enforceKinematicConstraints(
        std::vector<TrajectoryPoint>& trajectory) noexcept;

    double interpolateAngle(double start, double end, double t) noexcept;
    double quinticPolynomial(double t) const noexcept;
    double quinticPolynomialDerivative(double t) const noexcept;
    double quinticPolynomialSecondDerivative(double t) const noexcept;
    double normalizeAngle(double angle) const noexcept;
};