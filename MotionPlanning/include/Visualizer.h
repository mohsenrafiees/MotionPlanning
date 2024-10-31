
#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <Eigen/Dense>
#include <vector>
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

/**
 * A class to handle all the visualization for the path planning system.
 * This includes plotting the robot's path and velocity profiles.
 */
class Visualizer {
public:
    /**
     * Plots the planned path of the robot.
     * @param path: A vector of Eigen::Vector3d representing the robot's path (x, y, theta).
     */
    void plotPath(const std::vector<Eigen::Vector3d>& path) {
        std::vector<double> x_vals, y_vals, theta_vals;
        for (const auto& pose : path) {
            x_vals.push_back(pose.x());
            y_vals.push_back(pose.y());
            theta_vals.push_back(pose.z());
        }

        plt::plot(x_vals, y_vals, "r-");
        plt::xlabel("X [m]");
        plt::ylabel("Y [m]");
        plt::title("Planned Path");
        plt::show();
    }

    /**
     * Plots the linear and angular velocity profiles of the robot.
     * @param v: A vector of linear velocities over time.
     * @param omega: A vector of angular velocities over time.
     */
    void plotVelocities(const std::vector<double>& time, const std::vector<double>& v, const std::vector<double>& omega) {
        // Plot linear velocity
        plt::figure();
        plt::plot(time, v, "b-");
        plt::xlabel("Time [s]");
        plt::ylabel("Linear Velocity [m/s]");
        plt::title("Linear Velocity Profile");

        // Plot angular velocity
        plt::figure();
        plt::plot(time, omega, "g-");
        plt::xlabel("Time [s]");
        plt::ylabel("Angular Velocity [rad/s]");
        plt::title("Angular Velocity Profile");

        plt::show();
    }
};

#endif // VISUALIZER_HPP
