/*
 * Nolints in this header, unless otherwise stated, are due to naming conventions
 */
#ifndef DMPLIB_RIEMANN_MANIFOLDS_HPP
#define DMPLIB_RIEMANN_MANIFOLDS_HPP

#include <Eigen/Dense>

namespace dmp::riemannmanifold {

template <typename T>
struct tangent_space_dimension;

template <typename T>
struct tangent_space {
    using type = Eigen::Matrix<double, tangent_space_dimension<T>::value, 1>;  // NOLINT
};

template <typename T>
using tangent_space_t = tangent_space<T>::type;  // NOLINT

}  // namespace dmp::riemannmanifold


#endif  // DMPLIB_RIEMANN_MANIFOLDS_HPP
