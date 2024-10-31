#ifndef PLOTTING_UTILS_H
#define PLOTTING_UTILS_H

#include <vector>
#include <mutex>
#include <memory>
#include <Eigen/Dense>
#include "matplotlibcpp.h"
#include "common_defs.h"
#include "AStarPlanner.h"
#include "ThetaStarPlanner.h"
#include "SBPLPlanner.h"
#include "TrajectoryPlanner.h"
#include "RRTStarPlanner.h"
#include "CHOMPPlanner.h"
#include "STOMPPlanner.h"

namespace plt = matplotlibcpp;

class PlottingUtils {
public:
    // Function to create a planner instance based on type
    static std::unique_ptr<BasePlanner> createPlanner(PlannerType planner_type, 
                                                    const OccupancyGrid& grid, 
                                                    double robot_radius);
    // Function to plot path
    static void plotPath(const std::vector<Eigen::Vector3d>& path, const OccupancyGrid& grid, double robot_radius, const std::string& output_dir);
    // Function to plot trajectory
    static void plotTrajectory(const std::vector<TrajectoryPoint>& trajectory);

private:
    // Helper functions for plotting
    static void setupFigure();
    static void configurePlot(const std::string& title,
                            const std::string& xlabel,
                            const std::string& ylabel,
                            bool show_grid = true,
                            bool show_legend = true);
    static void initializePython();
    static std::mutex python_mutex;
    static bool python_initialized;
};

#endif // PLOTTING_UTILS_H