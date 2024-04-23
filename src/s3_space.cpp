#include "dmp/s3_space.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

using dmp::S3Space;

S3Space::Tangent S3Space::logarithmic_map(const Domain& x, const Domain& y) const {
    const Eigen::Map<const Eigen::Quaterniond> q1(x.data());
    const Eigen::Map<const Eigen::Quaterniond> q2(y.data());
    const Eigen::Quaterniond q = q2 * q1.conjugate();
    const Eigen::Map<const Domain> quat(q.coeffs().data());
    return logarithmic_map(quat);
}

S3Space::Tangent S3Space::logarithmic_map(const Domain& q) const{
    const Eigen::Map<const Eigen::Quaterniond> quat(q.data());
    const Eigen::Vector3d v = quat.vec();
    const double w = quat.w();
    return v;
}
