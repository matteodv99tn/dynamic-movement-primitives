#include "dmplib/manifolds/se3_manifold.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "dmplib/manifolds/rn_manifold.hpp"
#include "dmplib/manifolds/s3_manifold.hpp"

using dmp::SE3Manifold;

using dmp::SE3;  // domain
using Vec6 = dmp::SE3Manifold::Tangent_t;

Vec6
SE3Manifold::logarithmic_map_impl(const SE3& p, const SE3& x) const {
    Vec6 res;
    res.head<3>() = _r3_manifold.logarithmic_map(p.p, x.p);
    res.tail<3>() = _s3_manifold.logarithmic_map(p.q, x.q);
    return res;
}

SE3
SE3Manifold::exponential_map_impl(const SE3& p, const Vec6& v) const {
    return SE3(
            _r3_manifold.exponential_map(p.p, v.head<3>()),
            _s3_manifold.exponential_map(p.q, v.tail<3>())
    );
}
