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
    return dmp::logarithmic_map(dmp::quaternion_product(q1, q2.conjugate()));
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
    return dmp::quaternion_product(dmp::exponential_map(v), q0);
}

Eigen::Quaterniond dmp::quaternion_product(
        const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2
) {
    const double w1 = q1.w();
    const double w2 = q2.w();
    const Eigen::Vector3d v1 = q1.vec();
    const Eigen::Vector3d v2 = q2.vec();

    const double w = w1 * w2 - v1.dot(v2);
    const Eigen::Vector3d v = w1 * v2 + w2 * v1 + v1.cross(v2);
    // return Eigen::Quaterniond(w, v.x(), v.y(), v.z());
    return q1 * q2;
}
