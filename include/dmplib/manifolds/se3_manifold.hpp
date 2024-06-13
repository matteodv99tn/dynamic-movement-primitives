#ifndef DMPLIB_SE3_MANIFOLD_HPP
#define DMPLIB_SE3_MANIFOLD_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "dmplib/manifolds/aliases.hpp"
#include "dmplib/manifolds/riemann_manifold.hpp"

namespace dmp::riemannmanifold {

struct SE3 {  // NOLINT: naming convention
    Vec3_t       pos;
    Quaternion_t ori;

    SE3();
};

template <>
struct tangent_space_dimension<SE3> {
    static constexpr int value = 6;
};

Vec6_t       logarithmic_map(const SE3& q1, const SE3& q2);
Quaternion_t exponential_map(const SE3& q, const Vec6_t& v);

}  // namespace dmp::riemannmanifold

#endif  // DMPLIB_SE3_MANIFOLD_HPP
