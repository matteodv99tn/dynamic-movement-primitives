#include "dmp/quaternion_utils.hpp"

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

Eigen::Vector3d dmp::logarithmic_map(const Eigen::Quaterniond& q) {
    Eigen::Quaterniond q_normalized = q.normalized();
    Eigen::Vector3d    u(q_normalized.x(), q_normalized.y(), q_normalized.z());
    double             nu = q_normalized.w();

    double norm = 0;
    norm += u(0) * u(0);
    norm += u(1) * u(1);
    norm += u(2) * u(2);

    if (norm < 1e-12) return Eigen::Vector3d::Zero();

    Eigen::Vector3d res = std::acos(nu) * u / std::sqrt(norm);

    if (res.hasNaN()) {
        std::cout << "FOUND NAN - Input q: " << q << std::endl;
        std::cout << "norm: " << std::sqrt(norm) << std::endl;
        std::cout << "u: " << u << std::endl;
        std::cout << "acos: " << std::acos(nu) << std::endl;
    }

    return res;
}

Eigen::Vector3d dmp::logarithmic_map(
        const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2
) {
    return dmp::logarithmic_map(q1 * q2.conjugate());
}

Eigen::Quaterniond dmp::exponential_map(const Eigen::Vector3d& v) {
    if (v.norm() < 1e-6) return Eigen::Quaterniond::Identity();

    const double          v_norm = v.norm();
    const Eigen::Vector3d u      = std::sin(v_norm) * v / v_norm;
    Eigen::Quaterniond q_res (std::cos(v_norm), u(0), u(1), u(2));
    q_res.normalize();
    return q_res;
}

Eigen::Quaterniond dmp::exponential_map(
        const Eigen::Vector3d& v, const Eigen::Quaterniond& q0
) {
    return dmp::quaternion_product(dmp::exponential_map(v), q0);
}

Eigen::Quaterniond dmp::quaternion_product(
        const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2
) {
    const double          w1 = q1.w();
    const double          w2 = q2.w();
    const Eigen::Vector3d v1 = q1.vec();
    const Eigen::Vector3d v2 = q2.vec();

    const double          w = w1 * w2 - v1.dot(v2);
    const Eigen::Vector3d v = w1 * v2 + w2 * v1 + v1.cross(v2);
    // return Eigen::Quaterniond(w, v.x(), v.y(), v.z());
    return q1 * q2;
}

Eigen::MatrixXd dmp::quaternion_numerical_diff(
        const Eigen::MatrixXd& q_traj, const double& dt
) {
    Eigen::MatrixXd    q_ts(q_traj.rows(), 3);
    Eigen::Quaterniond qprev, qcurr;

    for (std::size_t i = 0; i < q_traj.rows(); i++) {
        qcurr = Eigen::Quaterniond(Eigen::Vector4d(q_traj.row(i)));
        if (i == 0) qprev = qcurr;

        const Eigen::Vector3d log = dmp::logarithmic_map(qcurr, qprev);
        q_ts.row(i)               = log;
        qprev                     = qcurr;
    }

    return 2 * q_ts / dt;
}

Eigen::MatrixXd dmp::rotate_angular_velocity(
        const Eigen::MatrixXd& omega, const Eigen::MatrixXd& q_traj
) {
    Eigen::MatrixXd omega_rot(omega.rows(), omega.cols());
    for (std::size_t i = 0; i < omega.rows(); ++i) {
        Eigen::Quaterniond qrot(Eigen::Vector4d(q_traj.row(i)));
        omega_rot.row(i) = qrot * omega.row(i);
    }
    return omega_rot;
}
