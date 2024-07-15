#ifndef DMP_RN_MANIFOLD_HPP
#define DMP_RN_MANIFOLD_HPP


#include "dmplib/manifolds/riemann_manifold.hpp"

namespace dmp::riemannmanifold {

template <int N>
struct tangent_space_dimension<VecN_t<N>> {
    static constexpr int value = N;
};

template <int N>
struct constants<VecN_t<N>> {
    static MatN_t<N>
    log_coefficient() {
        return MatN_t<N>::Identity();
    }

    static MatN_t<N>
    exp_coefficient() {
        return MatN_t<N>::Identity();
    }
};

template <int N>
VecN_t<N>
logarithmic_map(const VecN_t<N>& p1, const VecN_t<N>& p2) {
    return p2 - p1;
}

template <int N>
VecN_t<N>
exponential_map(const VecN_t<N>& p, const VecN_t<N>& v) {
    return p + v;
}

}  // namespace dmp::riemannmanifold

#endif  // DMP_RN_MANIFOLD_HPP
