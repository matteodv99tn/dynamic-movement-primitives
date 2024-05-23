#include "dmplib/manifolds/se3_manifold.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "dmplib/manifolds/rn_manifold.hpp"
#include "dmplib/manifolds/s3_manifold.hpp"

using dmp::SE3Manifold;

using dmp::SE3;  // domain
using Vec6_t = dmp::SE3Manifold::Tangent_t;

SE3::SE3(Eigen::Vector3d position, Eigen::Quaterniond orientation) :
        p(std::move(position)), q(std::move(orientation)) {
}

Vec6_t
SE3Manifold::logarithmic_map_impl(const SE3& p, const SE3& x) const {
    Vec6_t res;
    res.head<3>() = _r3_manifold.logarithmic_map(p.p, x.p);
    res.tail<3>() = _s3_manifold.logarithmic_map(p.q, x.q);
    return res;
}

SE3
SE3Manifold::exponential_map_impl(const SE3& p, const Vec6_t& v) const {
    return {_r3_manifold.exponential_map(p.p, v.head<3>()),
            _s3_manifold.exponential_map(p.q, v.tail<3>())};
}
