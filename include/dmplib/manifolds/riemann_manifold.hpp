/*
 * Nolints in this header, unless otherwise stated, are due to naming conventions
 */
#ifndef DMPLIB_RIEMANN_MANIFOLDS_HPP
#define DMPLIB_RIEMANN_MANIFOLDS_HPP

#include <concepts>
#include <Eigen/Dense>

namespace dmp::riemannmanifold {

template <int N>
using Vec_t = Eigen::Matrix<double, N, 1>;

using Vec2_t       = Vec_t<2>;  // NOLINT: are not magic numbers
using Vec3_t       = Vec_t<3>;  // NOLINT
using Vec6_t       = Vec_t<6>;  // NOLINT
using Quaternion_t = Eigen::Quaterniond;

template <typename T>
struct tangent_space_dimension;

template <typename T>
struct tangent_space {
    using type = Eigen::Matrix<double, tangent_space_dimension<T>::value, 1>;  // NOLINT
};

template <typename T>
using tangent_space_t = typename tangent_space<T>::type;  // NOLINT

template <typename T>
constexpr T default_constructor();


}  // namespace dmp::riemannmanifold

#include "dmplib/manifolds/rn_manifold.hpp"
#include "dmplib/manifolds/s3_manifold.hpp"
#include "dmplib/manifolds/se3_manifold.hpp"

namespace dmp::riemannmanifold {
template <std::default_initializable T>
constexpr T
default_constructor() {
    return T();
}

template <int N>
constexpr Eigen::Matrix<double, N, 1>
default_constructor() {
    return Eigen::Matrix<double, N, 1>::Zero();
}


}  // namespace dmp::riemannmanifold


#endif  // DMPLIB_RIEMANN_MANIFOLDS_HPP
