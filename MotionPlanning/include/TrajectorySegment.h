// TrajectorySegment.h

#ifndef TRAJECTORY_SEGMENT_H
#define TRAJECTORY_SEGMENT_H

#include <Eigen/Dense>

struct TrajectorySegment {
    Eigen::Vector2d start_position;
    Eigen::Vector2d end_position;
    double start_heading;
    double end_heading;
    double distance;
    double heading_change;

    double max_velocity;
    double max_acceleration;
    double max_jerk;

    double t_accel;        // Time to accelerate to max velocity
    double t_const;        // Time at constant velocity
    double t_decel;        // Time to decelerate to end velocity
    double total_time;     // Total time for the segment

    double v_initial;      // Initial velocity
    double v_final;        // Final velocity (for continuity)

    double a_initial;      // Initial acceleration
    double a_final;        // Final acceleration
};

#endif  // TRAJECTORY_SEGMENT_H
