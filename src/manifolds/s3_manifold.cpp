#include "dmplib/manifolds/s3_manifold.hpp"

#include <Eigen/Geometry>

#include "dmplib/utils/constants.hpp"


using namespace dmp::riemannmanifold;

Vec3_t
logarithmic_map(Quaternion_t q) {  // NOLINT
    if (q.w() < 0) q.coeffs() = -q.coeffs();
    const Eigen::Vector3d u  = q.vec();
    const double          nu = q.w();

    if (u.norm() < dmp::constants::zero_th) return Eigen::Vector3d::Zero();
    return std::acos(nu) * u.normalized();
}

Vec3_t
logarithmic_map(const Quaternion_t& qp, const Quaternion_t& qx) {
    return logarithmic_map_single(qx * qp.conjugate());
}

Quaternion_t
exponential_map(const Quaternion_t& p, const Vec3_t& v) {
    return exponential_map(v) * p;
}

Quaternion_t
exponential_map_single(const Vec3_t& v) {
    if (v.norm() < dmp::constants::zero_th) return Quaternion_t::Identity();

    Quaternion_t q;
    q.w()   = std::cos(v.norm());
    q.vec() = std::sin(v.norm()) * v.normalized();
    return q;
}
