#include "dmplib/manifolds/s3_manifold.hpp"

#include <Eigen/Geometry>

#include "dmplib/utils/constants.hpp"


using dmp::S3Manifold;
using Eigen::Quaterniond;  // domain
using Vec3_t = S3Manifold::Tangent_t;

Vec3_t
S3Manifold::logarithmic_map_single(Quaterniond q) {
    if (q.w() < 0) q.coeffs() = -q.coeffs();
    const Eigen::Vector3d u  = q.vec();
    const double          nu = q.w();

    if (u.norm() < dmp::constants::zero_th) return Eigen::Vector3d::Zero();
    return std::acos(nu) * u.normalized();
}

Vec3_t
S3Manifold::logarithmic_map_impl(const Quaterniond& qp, const Quaterniond& qx) {
    return logarithmic_map_single(qx * qp.conjugate());
}

Quaterniond
S3Manifold::exponential_map_impl(const Quaterniond& p, const Vec3_t& v) {
    return exponential_map_single(v) * p;
}

Quaterniond
S3Manifold::exponential_map_single(const Vec3_t& v) {
    if (v.norm() < dmp::constants::zero_th) return Quaterniond::Identity();

    Quaterniond q;
    q.w()   = std::cos(v.norm());
    q.vec() = std::sin(v.norm()) * v.normalized();
    return q;
}
