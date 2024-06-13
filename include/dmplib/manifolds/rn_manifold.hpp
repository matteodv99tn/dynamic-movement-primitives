#ifndef DMP_RN_MANIFOLD_HPP
#define DMP_RN_MANIFOLD_HPP


#include "dmplib/manifolds/aliases.hpp"
#include "dmplib/manifolds/riemann_manifold.hpp"

namespace dmp::riemannmanifold {

template <int N>
struct tangent_space_dimension<Vec_t<N>> {
    static constexpr int value = N;
};

template <int N>
Vec_t<N>
logarithmic_map(const Vec_t<N>& p1, const Vec_t<N>& p2) {
    return p2 - p1;
}

template <int N>
Vec_t<N>
exponential_map(const Vec_t<N>& p, const Vec_t<N>& v) {
    return p + v;
}

}  // namespace dmp::riemannmanifold

#endif  // DMP_RN_MANIFOLD_HPP
