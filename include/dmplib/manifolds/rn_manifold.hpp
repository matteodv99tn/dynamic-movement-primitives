#ifndef DMP_RN_MANIFOLD_HPP__
#define DMP_RN_MANIFOLD_HPP__

#include "dmplib/manifolds/riemann_manifolds.hpp"

namespace dmp {

// Manifold of a Rn space
template <int N>
class RnManifold
        : public RiemannManifold<RnManifold<N>, Eigen::Matrix<double, N, 1>, N> {
public:
    using Vec = Eigen::Matrix<double, N, 1>;

    static inline Vec
    construct_domain_impl() {
        return Vec::Zero();
    }

    Vec
    logarithmic_map_impl(const Vec& p, const Vec& x) const {
        return (x - p);
    }

    Vec
    exponential_map_impl(const Vec& p, const Vec& v) const {
        return p + v;
    }
};

class R3Manifold : public RnManifold<3> {};

}  // namespace dmp

#endif  // DMP_RN_MANIFOLD_HPP__
