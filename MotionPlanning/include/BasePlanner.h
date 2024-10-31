// BasePlanner.h

#ifndef BASEPLANNER_H
#define BASEPLANNER_H

#include <vector>
#include <queue>
#include <set>
#include <utility>  // for std::pair
#include <cmath>    // for M_PI
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <iostream>

class BasePlanner {
protected:
    virtual bool isCollisionFree(const Eigen::Vector2d& position) = 0;
    
    // Check if a straight line between two points is collision-free
    bool isPathFree(const Eigen::Vector2d& from, const Eigen::Vector2d& to, double resolution = 0.1) {
        Eigen::Vector2d diff = to - from;
        double distance = diff.norm();
        Eigen::Vector2d dir = diff.normalized();
        
        // Check points along the line
        for (double d = 0; d <= distance; d += resolution) {
            Eigen::Vector2d point = from + dir * d;
            if (!isCollisionFree(point)) {
                return false;
            }
        }
        return true;
    }
    
    // Store candidate points with their distances
    struct CandidatePoint {
        Eigen::Vector2d position;
        double distance;
        
        CandidatePoint(const Eigen::Vector2d& pos, double dist) 
            : position(pos), distance(dist) {}
        
        bool operator<(const CandidatePoint& other) const {
            return distance < other.distance;
        }
    };
    
    // Improved findClosestFreePoint that ensures reachability
    Eigen::Vector3d findClosestFreePoint(const Eigen::Vector3d& obstructed_point, 
                                       const Eigen::Vector3d& start_point,
                                       double search_radius = 3.0,    // Increased search radius
                                       double resolution = 0.2) {     // Step size
        std::vector<CandidatePoint> candidates;
        Eigen::Vector2d start_pos_2d = start_point.head<2>();
        Eigen::Vector2d goal_pos_2d = obstructed_point.head<2>();
        
        // First check along the direct line from start to goal
        Eigen::Vector2d dir = (goal_pos_2d - start_pos_2d).normalized();
        for (double d = 0.2; d <= (goal_pos_2d - start_pos_2d).norm(); d += resolution) {
            Eigen::Vector2d test_point = goal_pos_2d - dir * d;
            if (isCollisionFree(test_point) && isPathFree(start_pos_2d, test_point)) {
                std::cout << "Found free and reachable point along direct path, distance from goal: " 
                         << d << " meters" << std::endl;
                return Eigen::Vector3d(test_point.x(), test_point.y(), obstructed_point.z());
            }
        }

        // Search in expanding circles around the goal
        for (double r = resolution; r <= search_radius; r += resolution) {
            for (double angle = 0; angle < 2 * M_PI; angle += M_PI/8) {
                Eigen::Vector2d test_point(
                    goal_pos_2d.x() + r * cos(angle),
                    goal_pos_2d.y() + r * sin(angle)
                );
                
                if (isCollisionFree(test_point) && isPathFree(start_pos_2d, test_point)) {
                    candidates.emplace_back(test_point, r);
                }
            }
            
            // If we found any valid points at this radius, use the first one
            // (they're all roughly the same distance from the goal)
            if (!candidates.empty()) {
                std::sort(candidates.begin(), candidates.end());
                auto& best = candidates[0];
                std::cout << "Found free and reachable point at distance: " 
                         << best.distance << " meters from goal" << std::endl;
                return Eigen::Vector3d(best.position.x(), best.position.y(), obstructed_point.z());
            }
        }
        
        // If still no point found, try a grid search
        for (double dx = -search_radius; dx <= search_radius; dx += resolution * 2) {
            for (double dy = -search_radius; dy <= search_radius; dy += resolution * 2) {
                if (dx*dx + dy*dy > search_radius*search_radius) continue;
                
                Eigen::Vector2d test_point(
                    goal_pos_2d.x() + dx,
                    goal_pos_2d.y() + dy
                );
                
                if (isCollisionFree(test_point) && isPathFree(start_pos_2d, test_point)) {
                    double dist = sqrt(dx*dx + dy*dy);
                    std::cout << "Found free and reachable point at distance: " 
                             << dist << " meters from goal" << std::endl;
                    return Eigen::Vector3d(test_point.x(), test_point.y(), obstructed_point.z());
                }
            }
        }

        std::cerr << "Warning: No reachable free point found within " << search_radius 
                  << " meters of obstructed goal." << std::endl;
        return obstructed_point;
    }
    
    // Updated helper method to validate goal
    Eigen::Vector3d validateGoal(const Eigen::Vector3d& goal, const Eigen::Vector3d& start) {
        if (!isCollisionFree(goal.head<2>())) {
            std::cout << "Goal position is in collision, searching for nearby free point..." << std::endl;
            return findClosestFreePoint(goal, start);
        }
        return goal;
    }

    std::vector<Eigen::Vector3d> smoothPath(const std::vector<Eigen::Vector3d>& path_points) const{
        int n = path_points.size();
        if (n < 3) return path_points; // Not enough points for smoothing

        std::vector<Eigen::Vector3d> smoothed_path;
        
        // Setup for spline: each segment between points will have its own cubic polynomial
        Eigen::VectorXd x(n), y(n);
        for (int i = 0; i < n; ++i) {
            x[i] = path_points[i].x();
            y[i] = path_points[i].y();
        }

        // Build the system of equations for cubic splines
        Eigen::SparseMatrix<double> A(4 * (n - 1), 4 * (n - 1));
        Eigen::VectorXd bx(4 * (n - 1)), by(4 * (n - 1));

        // Set up equations for each segment
        for (int i = 0; i < n - 1; ++i) {
            // Position continuity
            A.insert(4 * i, 4 * i) = 1;
            bx[4 * i] = x[i];
            by[4 * i] = y[i];
            
            // Position continuity for end of segment
            A.insert(4 * i + 1, 4 * i) = 1;
            A.insert(4 * i + 1, 4 * i + 1) = 1;
            A.insert(4 * i + 1, 4 * i + 2) = 1;
            A.insert(4 * i + 1, 4 * i + 3) = 1;
            bx[4 * i + 1] = x[i + 1];
            by[4 * i + 1] = y[i + 1];
            
            if (i < n - 2) {
                // First derivative continuity
                A.insert(4 * i + 2, 4 * i + 1) = 1;
                A.insert(4 * i + 2, 4 * i + 2) = 2;
                A.insert(4 * i + 2, 4 * i + 3) = 3;
                A.insert(4 * i + 2, 4 * (i + 1) + 1) = -1;
                bx[4 * i + 2] = 0;
                by[4 * i + 2] = 0;
                
                // Second derivative continuity
                A.insert(4 * i + 3, 4 * i + 2) = 2;
                A.insert(4 * i + 3, 4 * i + 3) = 6;
                A.insert(4 * i + 3, 4 * (i + 1) + 2) = -2;
                bx[4 * i + 3] = 0;
                by[4 * i + 3] = 0;
            }
        }

        // Solve the linear systems for x and y spline coefficients
        Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
        solver.analyzePattern(A);   // Analyze the sparsity pattern of A
        solver.factorize(A);        // Perform the numerical factorization

        if (solver.info() != Eigen::Success) {
            std::cerr << "Factorization failed! Falling back to linear interpolation." << std::endl;
            for (int i = 0; i < n - 1; ++i) {
                Eigen::Vector3d p0 = path_points[i];
                Eigen::Vector3d p1 = path_points[i + 1];
                
                // Generate linearly interpolated points between p0 and p1
                for (double t = 0; t <= 1; t += 0.05) { // Adjust step as needed
                    double xt = p0.x() + t * (p1.x() - p0.x());
                    double yt = p0.y() + t * (p1.y() - p0.y());
                    smoothed_path.emplace_back(xt, yt, 0);  // Assuming z=0
                }
            }
            return smoothed_path;
        }

        Eigen::VectorXd cx = solver.solve(bx);
        Eigen::VectorXd cy = solver.solve(by);

        // Check if the solve was successful
        if (solver.info() != Eigen::Success) {
            std::cerr << "Solving failed! Falling back to linear interpolation." << std::endl;
            // Repeat the linear interpolation fallback if solving fails
            for (int i = 0; i < n - 1; ++i) {
                Eigen::Vector3d p0 = path_points[i];
                Eigen::Vector3d p1 = path_points[i + 1];
                
                for (double t = 0; t <= 1; t += 0.05) {
                    double xt = p0.x() + t * (p1.x() - p0.x());
                    double yt = p0.y() + t * (p1.y() - p0.y());
                    smoothed_path.emplace_back(xt, yt, 0);
                }
            }
            return smoothed_path;
        }

        // Construct smoothed path based on the spline coefficients
        for (int i = 0; i < n - 1; ++i) {
            for (double t = 0; t <= 1; t += 0.05) {
                double xt = cx[4 * i] + cx[4 * i + 1] * t + cx[4 * i + 2] * t * t + cx[4 * i + 3] * t * t * t;
                double yt = cy[4 * i] + cy[4 * i + 1] * t + cy[4 * i + 2] * t * t + cy[4 * i + 3] * t * t * t;
                smoothed_path.emplace_back(xt, yt, 0);  // Assuming z=0
            }
        }

        return smoothed_path;
    }
public:
    virtual ~BasePlanner() = default;
    
    // Modified base method that includes goal validation
    virtual std::vector<Eigen::Vector3d> getCollisionFreePath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal) = 0;
    
    // Original smoothPath method remains unchanged
    
};

#endif // BASEPLANNER_H