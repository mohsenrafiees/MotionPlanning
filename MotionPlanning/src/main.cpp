#include "BasePlanner.h"
#include "SBPLPlanner.h"
#include "AStarPlanner.h"
#include "ThetaStarPlanner.h"
#include "RRTStarPlanner.h"
#include "OccupancyGrid.h"
#include "matplotlibcpp.h"
#include "TrajectoryPlanner.h"
#include "PathProfiler.h"
#include "plotting_utils.h"
#include "common_defs.h"
#include <iostream>
#include <vector>
#include <map>
#include <chrono>
#include <memory>
#include <filesystem>
#include <sys/resource.h>
#include <fstream>
#include <ctime>

namespace plt = matplotlibcpp;
namespace fs = std::filesystem;

// Custom logger class
class Logger {
public:
    Logger(const std::string& filename) {
        log_file.open(filename, std::ios::app);
    }

    template<typename T>
    void info(const T& message) {
        log("INFO", message);
    }

    template<typename T>
    void error(const T& message) {
        log("ERROR", message);
    }

    template<typename T>
    void debug(const T& message) {
        log("DEBUG", message);
    }

private:
    std::ofstream log_file;

    template<typename T>
    void log(const std::string& level, const T& message) {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        std::string timestamp = std::ctime(&time);
        timestamp.pop_back(); // Remove newline

        log_file << "[" << timestamp << "] [" << level << "] " << message << std::endl;
        std::cout << "[" << level << "] " << message << std::endl;
    }
};

// Helper function to get current memory usage
size_t getCurrentMemoryUsage() {
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    return usage.ru_maxrss * 1024; // Convert to bytes
}

// Helper function to ensure output directory exists
void ensureDirectoryExists(const std::string& path) {
    if (!fs::exists(path)) {
        fs::create_directories(path);
    }
}

// Function to run planner and gather metrics
std::tuple<std::vector<Eigen::Vector3d>, double, size_t> 
runPlannerWithMetrics(BasePlanner* planner, 
                     const Eigen::Vector3d& start, 
                     const Eigen::Vector3d& goal) {
    // Record initial memory
    size_t initial_memory = getCurrentMemoryUsage();
    
    // Start timing
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Run planner
    std::cout << "Planner is running" << std::endl;
    auto path = planner->getCollisionFreePath(start, goal);
    
    // End timing
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    // Calculate memory usage
    size_t final_memory = getCurrentMemoryUsage();
    size_t memory_used = final_memory - initial_memory;
    
    return {path, duration.count(), memory_used};
}

int main() {
    // Initialize logging
    ensureDirectoryExists("logs");
    Logger logger("logs/motion_planner.log");
    
    try {
        // Create output directories
        std::string output_base = "output";
        std::string metrics_dir = output_base + "/metrics";
        std::string plots_dir = output_base + "/plots";
        ensureDirectoryExists(metrics_dir);
        ensureDirectoryExists(plots_dir);

        // Load and prepare occupancy grid
        OccupancyGrid grid;
        double inflation_radius_m = 0.02;
        double map_resolution_m = 0.05;
        grid.loadFromImage("grid-example.png", map_resolution_m, inflation_radius_m);

        // Robot parameters
        double robot_radius = 0.2;
        double inflated_radius = 0.02;

        // Planning points
        // Eigen::Vector3d start(5, 14, 0);
        // Eigen::Vector3d goal(12, 4, 0);

        // ---- Case 0 -----
        // Eigen::Vector3d start(5, 14, 0);
        // Eigen::Vector3d goal(12, 4, 0);

        // ---- Case 1 -----
        // Obstructed goal
        // Eigen::Vector3d start(5, 14, 0);
        // Eigen::Vector3d goal(6, 8, 0);

        // ---- Case 2 -----
        // Stright Line without obstruction
        Eigen::Vector3d start(2, 2, 0);
        Eigen::Vector3d goal(12, 14, 0);

        // Create path profiler
        PathProfiler profiler(grid, robot_radius + inflated_radius);

        // Initialize trajectory planner
        double max_linear_vel = 1.0; // (m/s)
        double max_linear_acc = 0.7; // (m/s^2)
        double max_angular_vel = 0.85; // (rad/s)
        double max_angular_acc = 2.0; // (rad/s^2)
        double max_jerk = 1.0; // (m/s^3)
        double time_resolution = 0.05;
        double min_velocity = 0.0;
        double min_turning_radius = 0.2;

        TrajectoryPlanner trajectory_planner(
            max_linear_vel, max_linear_acc,
            max_angular_vel, max_angular_acc, 
            time_resolution, min_turning_radius,
            0.5,  // caution_threshold in meters
            0.1   // merge_threshold for dynamics
        );

        // Test different planners and gather metrics
        std::vector<PlannerType> planner_types = {
            PlannerType::SBPL,
            PlannerType::AStar,
            PlannerType::RRTStar,
            // PlannerType::CHOMPPlanner
            //PlannerType::STOMPPlanner
            //PlannerType::ThetaStar
        };

        std::map<PlannerType, std::string> planner_names = {
            {PlannerType::SBPL, "SBPL"},
            {PlannerType::AStar, "A*"},
            {PlannerType::ThetaStar, "Theta*"},
            {PlannerType::RRTStar, "RRT*"},
            {PlannerType::CHOMPPlanner, "CHOMPP"},
            {PlannerType::STOMPPlanner, "STOMP"}
        };

        // Results container
            struct PlannerResults {
        std::vector<Eigen::Vector3d> path;
        std::vector<TrajectoryPoint> trajectory;
        PathMetrics path_metrics;
        TrajectoryMetrics traj_metrics;

        // Add constructor
        PlannerResults() = default;
        
        // Add constructor that takes all members
        PlannerResults(const std::vector<Eigen::Vector3d>& p,
                      const std::vector<TrajectoryPoint>& t,
                      const PathMetrics& pm,
                      const TrajectoryMetrics& tm)
            : path(p), trajectory(t), path_metrics(pm), traj_metrics(tm) {}
        };
        std::map<PlannerType, PlannerResults> results;

        // Test each planner
        for (const auto& planner_type : planner_types) {
            logger.info("Testing " + planner_names[planner_type] + " planner");
            
            // Create planner
            std::cout << "About to run PlottingUtils::createPlanner: " << std::endl;
            auto planner = PlottingUtils::createPlanner(planner_type, grid, robot_radius);
            
            // Run planner and gather metrics
            auto [path, planning_time, memory_used] = runPlannerWithMetrics(
                planner.get(), start, goal
            );

            if (path.empty()) {
                logger.error(planner_names[planner_type] + " planner failed to find a path");
                continue;
            }

            // Generate trajectory
            std::vector<TrajectoryPoint> trajectory;
            trajectory_planner.generateSmoothTrajectory(grid, path, trajectory, robot_radius, time_resolution);

         

            PathMetrics path_metrics = profiler.profilePath(path, planning_time, memory_used);
            TrajectoryMetrics traj_metrics = profiler.profileTrajectory(
                trajectory, max_linear_vel, max_linear_acc,
                max_angular_vel, max_angular_acc
            );

            // Store results
            results[planner_type] = {path, trajectory, path_metrics, traj_metrics};

            // Generate visualizations
            std::string planner_dir = metrics_dir + "/" + planner_names[planner_type];
            ensureDirectoryExists(planner_dir);
            profiler.plotTrajectory(trajectory, planner_dir);
            profiler.visualizeMetrics(path_metrics, traj_metrics, planner_dir);

            try {
                if (!path.empty()) {
                    PlottingUtils::plotPath(path, grid, robot_radius, planner_dir); // Pass planner_dir as output_dir
                    
                    if (!trajectory.empty()) {
                        PlottingUtils::plotTrajectory(trajectory);

                        // Move trajectory plots to the correct directory
                        std::vector<std::string> trajectory_files = {
                            "velocity_profile.png",
                            "acceleration_profile.png",
                            "trajectory_path.png"
                        };

                        for (const auto& file : trajectory_files) {
                            if (std::filesystem::exists(file)) {
                                std::filesystem::rename(file, planner_dir + "/" + file);
                            }
                        }
                    } else {
                        std::cout << "Trajectory is Empty" << std::endl;
                    }
                } else {
                    std::cout << "Path is Empty" << std::endl;
                }
            } catch (const std::exception& e) {
                logger.error("Plotting failed: " + std::string(e.what()));
            }

            // Log results
            logger.info(planner_names[planner_type] + " Results:");
            logger.info("  Path length: " + std::to_string(path_metrics.total_length) + "m");
            logger.info("  Computation time: " + std::to_string(path_metrics.computation_time_ms) + "ms");
            logger.info("  Memory usage: " + std::to_string(path_metrics.memory_usage_bytes / 1024.0 / 1024.0) + "MB");
            logger.info("  Safety score: " + std::to_string(path_metrics.safety_score));
            logger.info("  Smoothness: " + std::to_string(path_metrics.smoothness));
        }

        // Compare planners
        std::ofstream comparison(metrics_dir + "/planner_comparison.txt");
        comparison << "Planner Comparison Results\n";
        comparison << "=========================\n\n";

        for (const auto& [planner_type, result] : results) {
            comparison << planner_names[planner_type] << ":\n";
            comparison << "-----------------\n";
            comparison << "Path score: " << result.path_metrics.path_score << "\n";
            comparison << "Path length: " << result.path_metrics.total_length << "m\n";
            comparison << "Computation time: " << result.path_metrics.computation_time_ms << "ms\n";
            comparison << "Memory usage: " << result.path_metrics.memory_usage_bytes << "B\n";
            comparison << "Safety score: " << result.path_metrics.safety_score << "\n";
            comparison << "Smoothness: " << result.path_metrics.smoothness << "\n";
            comparison << "Trajectory duration: " << result.traj_metrics.total_duration << "s\n";
            comparison << "Average velocity: " << result.traj_metrics.average_velocity << "m/s\n";
            comparison << "Constraints met: " 
                      << (result.traj_metrics.velocity_constraints_met ? "Yes" : "No") << "\n\n";
        }

    } catch (const std::exception& e) {
        logger.error(std::string("Error: ") + e.what());
        return -1;
    }

    return 0;
}