#ifndef DMP_RN_MANIFOLD_HPP__
#define DMP_RN_MANIFOLD_HPP__

#include "dmp/manifolds/riemann_manifolds.hpp"

namespace dmp{

    // Manifold of a Rn space
    template <int N>
    class RnManifold : public RiemannianManifold<RnManifold<N>, Eigen::Matrix<double, N, 1>, N> {
    public:

        using Vec = Eigen::Matrix<double, N, 1>;

        Vec construct_domain_impl() const {
            return Vec::Zero();
        }

        Vec logarithmic_map_impl(const Vec& application_point, const Vec& y) const {
            return y - application_point;
        }
        
        Vec exponential_map_impl(const Vec& application_point, const Vec& v) const {
            return application_point + v;
        }
    };

}

#endif // DMP_RN_MANIFOLD_HPP__