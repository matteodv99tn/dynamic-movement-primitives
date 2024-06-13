#ifndef DMPLIB_RIEMANN_MANIFOLD_CONCEPTS_HPP
#define DMPLIB_RIEMANN_MANIFOLD_CONCEPTS_HPP

#include <concepts>

#include "dmplib/manifolds/riemann_manifold.hpp"

namespace dmp::riemannmanifold::concepts {

namespace rm = ::dmp::riemannmanifold;


template <typename T>
concept with_tangent_type = requires { typename tangent_space_t<T>; };

template <typename T>
concept with_logarithmic_map = requires(T p1, T p2) {
    { rm::logarithmic_map(p1, p2) } -> std::same_as<tangent_space_t<T>>;
};


// template <typename T>
// concept riemann_manifold = with_tangent_type<T> && with_logarithmic_map<T>;


}  // namespace dmp::riemannmanifold::concepts


#endif  // DMPLIB_RIEMANN_MANIFOLD_CONCEPTS_HPP
