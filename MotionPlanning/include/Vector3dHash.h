#ifndef VECTOR3DHASH_H
#define VECTOR3DHASH_H

#include <Eigen/Dense>

struct Vector3dHash {
    std::size_t operator()(const Eigen::Vector3d& v) const {
        std::size_t hx = std::hash<double>{}(v.x());
        std::size_t hy = std::hash<double>{}(v.y());
        std::size_t hz = std::hash<double>{}(v.z());
        return hx ^ (hy << 1) ^ (hz << 2);
    }
};

#endif // VECTOR3DHASH_H
