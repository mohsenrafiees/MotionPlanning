// OccupancyGrid.h

#ifndef OCCUPANCYGRID_H
#define OCCUPANCYGRID_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

class OccupancyGrid {
public:
    int width;
    int height;
    int origin_x;
    int origin_y;
    double resolution_m; // Resolution in meters per cell
    std::vector<bool> data;

    void loadFromImage(const std::string& filename, double resolution_m_input, double inflation_radius_m);
    bool isCollision(const Eigen::Vector2d& position, double robot_radius) const;
    cv::Mat computeDistanceMap() const;
    double getObstacleCost(double x, double y) const; 
    double distanceToNearestObstacle(const Eigen::Vector3d& position) const;
    double getDistanceToNearestObstacle(const Eigen::Vector2d& point) const {
        // Wrapper to convert Vector2d to Vector3d for the existing function
        return distanceToNearestObstacle(Eigen::Vector3d(point.x(), point.y(), 0.0));
    }
};

#endif // OCCUPANCYGRID_H
