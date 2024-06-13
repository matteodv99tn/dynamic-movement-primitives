#ifndef DMPLIB_S3_MANIFOLD_HPP
#define DMPLIB_S3_MANIFOLD_HPP

#include <Eigen/Geometry>

#include "dmplib/manifolds/aliases.hpp"
#include "dmplib/manifolds/riemann_manifold.hpp"

namespace dmp::riemannmanifold {

template <>
struct tangent_space_dimension<Quaternion_t> {
    static constexpr int value = 3;
};

Vec3_t       logarithmic_map(const Quaternion_t& q1, const Quaternion_t& q2);
Vec3_t       logarithmic_map_single(Quaternion_t q);
Quaternion_t exponential_map(const Vec3_t& v);
Quaternion_t exponential_map(const Quaternion_t& q, const Vec3_t& v);

}  // namespace dmp::riemannmanifold

#endif  // DMPLIB_S3_MANIFOLD_HPP
