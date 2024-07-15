#ifndef DMPLIB_S3_MANIFOLD_HPP
#define DMPLIB_S3_MANIFOLD_HPP

#include "dmplib/manifolds/aliases.hpp"
#include "dmplib/manifolds/riemann_manifold.hpp"

namespace dmp::riemannmanifold {

template <>
struct tangent_space_dimension<Quaternion_t> {
    static constexpr int value = 3;
};

template <>
struct constants<Quaternion_t> {
    static Mat3_t
    log_coefficient() {
        return 2.0 * Mat3_t::Identity();
    }
    
    static Mat3_t
    exp_coefficient() {
        return 0.5 * Mat3_t::Identity();
    }
};

Vec3_t       logarithmic_map(const Quaternion_t& q1, const Quaternion_t& q2);
Vec3_t       logarithmic_map_single(Quaternion_t q);
Quaternion_t exponential_map(const Quaternion_t& q, const Vec3_t& v);
Quaternion_t exponential_map_single(const Vec3_t& v);

template <typename T>
constexpr T
default_constructor() requires std::is_same_v<Quaternion_t, T> {
    return Quaternion_t::Identity();
}

}  // namespace dmp::riemannmanifold

#endif  // DMPLIB_S3_MANIFOLD_HPP
