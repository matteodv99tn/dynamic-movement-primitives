#ifndef DMP_RN_MANIFOLD_HPP
#define DMP_RN_MANIFOLD_HPP

#include "dmplib/manifolds/riemann_manifolds.hpp"

namespace dmp {

// Manifold of a Rn space
template <int N>
class RnManifold
        : public RiemannManifold<RnManifold<N>, Eigen::Matrix<double, N, 1>, N> {
public:
    using Vec_t = Eigen::Matrix<double, N, 1>;

    [[nodiscard]] inline Vec_t
    construct_domain_impl() const {
        return Vec_t::Zero();
    }

    [[nodiscard]] Vec_t
    logarithmic_map_impl(const Vec_t& p, const Vec_t& x) const {
        return (x - p);
    }

    [[nodiscard]] Vec_t
    exponential_map_impl(const Vec_t& p, const Vec_t& v) const {
        return p + v;
    }
};

class R3Manifold : public RnManifold<3> {};

}  // namespace dmp

#endif  // DMP_RN_MANIFOLD_HPP
