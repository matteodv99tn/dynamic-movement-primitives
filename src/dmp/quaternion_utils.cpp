#include "dmp/quaternion_utils.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

Eigen::Vector3d dmp::logarithmic_map(const Eigen::Quaterniond& q) {

    Eigen::Vector3d u(q.x(), q.y(), q.z());
    double nu = q.w();

    if (u.norm() < 1e-6)
        return Eigen::Vector3d::Zero();

    return std::acos(nu) * u / u.norm();
}

Eigen::Vector3d dmp::logarithmic_map(
        const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2
) {
    return dmp::logarithmic_map(q1 * q2.conjugate());
}

Eigen::Quaterniond dmp::exponential_map(const Eigen::Vector3d& v) {
    if (v.norm() < 1e-6)
        return Eigen::Quaterniond::Identity();

    const double v_norm = v.norm();
    const Eigen::Vector3d u = std::sin(v_norm) * v / v_norm;

    return Eigen::Quaterniond(std::cos(v_norm), u(0), u(1), u(2));
}

Eigen::Quaterniond dmp::exponential_map(
        const Eigen::Vector3d& v, const Eigen::Quaterniond& q0
) {
    return dmp::exponential_map(v) * q0;
}
