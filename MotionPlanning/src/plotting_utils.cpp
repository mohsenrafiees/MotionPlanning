#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <Python.h>
#include <numpy/arrayobject.h>
#include "matplotlibcpp.h"
#include "plotting_utils.h"
#include <opencv2/opencv.hpp>

namespace plt = matplotlibcpp;

// Define static members
std::mutex PlottingUtils::python_mutex;
bool PlottingUtils::python_initialized = false;

// Helper function for NumPy initialization
static void* init_numpy() {
    import_array();
    return nullptr;
}

void PlottingUtils::initializePython() {
    std::lock_guard<std::mutex> lock(python_mutex);
    if (!python_initialized) {
        try {
            if (!Py_IsInitialized()) {
                Py_Initialize();
                
                // Initialize Python components in correct order
                const char* setup_code = R"(
                                            import sys
                                            import numpy as np
                                            import matplotlib
                                            matplotlib.use('TkAgg')
                                            import matplotlib.pyplot as plt
                                            plt.ion()  # Enable interactive mode
                                            )";
                if (PyRun_SimpleString(setup_code) != 0) {
                    throw std::runtime_error("Failed to initialize Python/Matplotlib");
                }
            }
            
            // Initialize NumPy
            init_numpy();
            
            python_initialized = true;
            
        } catch (const std::exception& e) {
            std::cerr << "Failed to initialize Python/NumPy: " << e.what() << std::endl;
            throw;
        }
    }
}

void PlottingUtils::setupFigure() {
    try {
        // Create new figure
        plt::figure();
        
        // Set the figure size
        plt::figure_size(1200, 800);
        
    } catch (const std::exception& e) {
        std::cerr << "Error in setupFigure: " << e.what() << std::endl;
        throw;
    }
}

// Rest of the implementation stays the same...

std::unique_ptr<BasePlanner> PlottingUtils::createPlanner(PlannerType planner_type, 
                                                         const OccupancyGrid& grid, 
                                                         double robot_radius) {
    std::cout << "About to run PlottingUtils::createPlanner: " << std::endl;
    switch (planner_type) {
        case PlannerType::AStar:
            return std::make_unique<AStarPlanner>(grid, robot_radius);
        case PlannerType::ThetaStar:
            return std::make_unique<ThetaStarPlanner>(grid, robot_radius);
        case PlannerType::SBPL:
            return std::make_unique<SBPLPlanner>(grid, robot_radius);
        case PlannerType::RRTStar:
            return std::make_unique<RRTStarPlanner>(grid, robot_radius);
        case PlannerType::CHOMPPlanner:
            return std::make_unique<CHOMPPlanner>(grid, robot_radius);
        case PlannerType::STOMPPlanner:
            return std::make_unique<STOMPPlanner>(grid, robot_radius);
        default:
            throw std::invalid_argument("Invalid planner type selected!");
    }
}


void PlottingUtils::plotPath(const std::vector<Eigen::Vector3d>& path, const OccupancyGrid& grid, double robot_radius, const std::string& output_dir) {
    if (path.empty()) {
        std::cerr << "Warning: Empty path provided to plotPath" << std::endl;
        return;
    }

    // Initialize Python environment if needed
    PlottingUtils::initializePython();
    std::lock_guard<std::mutex> lock(python_mutex);  // Ensure thread-safe plotting

    try {
        // Prepare obstacle coordinates
        std::stringstream obstacle_stream;
        obstacle_stream << R"(
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle

map_x, map_y = [], []
)";

        for (int y = 0; y < grid.height; ++y) {
            for (int x = 0; x < grid.width; ++x) {
                if (grid.data[y * grid.width + x]) {
                    double real_x = x * grid.resolution_m;
                    double real_y = (grid.height - y - 1) * grid.resolution_m;
                    obstacle_stream << "map_x.append(" << real_x << ")\n";
                    obstacle_stream << "map_y.append(" << real_y << ")\n";
                }
            }
        }

        // Plot obstacles
        obstacle_stream << R"(
plt.figure(figsize=(10,10))
plt.scatter(map_x, map_y, c='black', s=10, marker='s')
)";
        PyRun_SimpleString(obstacle_stream.str().c_str());

        // Extract path coordinates
        std::vector<double> x_path(path.size()), y_path(path.size());
        for (size_t i = 0; i < path.size(); ++i) {
            x_path[i] = path[i].x();
            y_path[i] = path[i].y();
        }

        // Set axis limits
        plt::xlim(0.0, grid.width * grid.resolution_m);
        plt::ylim(0.0, grid.height * grid.resolution_m);

        // Plot path
        plt::plot(x_path, y_path, {{"label", "Path"}, {"color", "blue"}, {"linewidth", "3"}});
        plt::plot({x_path[0]}, {y_path[0]}, {{"marker", "o"}, {"color", "green"}, {"markersize", "12"}, {"label", "Start"}});
        plt::plot({x_path.back()}, {y_path.back()}, {{"marker", "o"}, {"color", "red"}, {"markersize", "12"}, {"label", "Goal"}});

        // Add robot footprint circles along the path
        obstacle_stream.str("");  // Clear the stream
        for (size_t i = 0; i < x_path.size(); ++i) {
            obstacle_stream << "plt.gca().add_patch(Circle((" << x_path[i] << ", " << y_path[i] << "), "
                            << robot_radius << ", facecolor='orange', edgecolor='black', alpha=0.5))\n";
        }
        PyRun_SimpleString(obstacle_stream.str().c_str());

        // Set plot labels, grid, and legend
        plt::xlabel("X (meters)");
        plt::ylabel("Y (meters)");
        plt::title("Path Planning Result with Robot Footprint");
        plt::axis("equal");
        plt::grid(true);
        plt::legend();

        // Save plot to specified directory
        std::string filepath = output_dir + "/path_planning_result_with_footprint.png";
        plt::save(filepath);
        plt::close();
        std::cout << "Successfully saved path plot with robot footprint as '" << filepath << "'" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Plotting failed: " << e.what() << std::endl;
        PyRun_SimpleString("plt.close()");  // Cleanup
    }
}


void PlottingUtils::plotTrajectory(const std::vector<TrajectoryPoint>& trajectory) {
    if (trajectory.empty()) {
        std::cerr << "Warning: Empty trajectory provided to plotTrajectory" << std::endl;
        return;
    }

    // Initialize Python environment if needed
    PlottingUtils::initializePython();
    std::lock_guard<std::mutex> lock(python_mutex);  // Ensure thread safety for plotting

    try {
        // Vectors to store data
        std::vector<double> times, v_vals, lin_accels;
        std::vector<double> x_vals, y_vals, headings;
        
        times.reserve(trajectory.size());
        v_vals.reserve(trajectory.size());
        lin_accels.reserve(trajectory.size());
        x_vals.reserve(trajectory.size());
        y_vals.reserve(trajectory.size());
        headings.reserve(trajectory.size());

        // Populate the vectors
        for (const auto& point : trajectory) {
            times.push_back(point.time);
            v_vals.push_back(point.velocity);
            lin_accels.push_back(point.acceleration);
            x_vals.push_back(point.position.x());
            y_vals.push_back(point.position.y());
            headings.push_back(point.heading);
        }

        if (times.size() != trajectory.size()) {
            std::cerr << "Error: Vector sizes do not match trajectory size" << std::endl;
            return;
        }

        // Plotting code
        {
            PlottingUtils::setupFigure();
            plt::plot(times, v_vals, {{"label", "Linear Velocity (v)"}, {"color", "red"}});
            plt::xlabel("Time (s)");
            plt::ylabel("Velocity (m/s)");
            plt::legend();
            plt::title("Velocity Profile");
            plt::save("velocity_profile.png");
            plt::close();
        }

        {
            PlottingUtils::setupFigure();
            plt::plot(times, lin_accels, {{"label", "Linear Acceleration"}, {"color", "blue"}});
            plt::xlabel("Time (s)");
            plt::ylabel("Acceleration (m/s²)");
            plt::legend();
            plt::title("Acceleration Profile");
            plt::save("acceleration_profile.png");
            plt::close();
        }

        {
            PlottingUtils::setupFigure();
            plt::plot(x_vals, y_vals, {{"label", "Trajectory"}, {"color", "purple"}});
            plt::xlabel("X Position (m)");
            plt::ylabel("Y Position (m)");

            std::vector<double> u_vals, v_headings;
            double arrow_length = 0.5;

            for (const auto& heading : headings) {
                u_vals.push_back(arrow_length * std::cos(heading));
                v_headings.push_back(arrow_length * std::sin(heading));
            }

            plt::quiver(x_vals, y_vals, u_vals, v_headings, {{"color", "green"}});
            plt::axis("equal");
            plt::legend();
            plt::title("Trajectory Path with Heading");
            plt::save("trajectory_path.png");
            plt::close();
        }

        std::cout << "Successfully saved all trajectory plots" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error in plotTrajectory: " << e.what() << std::endl;
        PyRun_SimpleString("plt.close()");
    }
}