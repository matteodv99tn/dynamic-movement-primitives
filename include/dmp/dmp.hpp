#ifndef DMP_BASE_HPP__
#define DMP_BASE_HPP__

#include "dmp/manifolds/riemann_manifolds.hpp"
#include "dmp/traits.hpp"

namespace dmp {

    const int DiscreteDmp = 1;
    const int PeriodicDmp = 2;

    template <dmp::traits::riemann_manifold Manifold, int DmpType = PeriodicDmp>
    class Dmp : public Manifold {
    public:
        using Manifold_t = Manifold;

        Dmp() {
            using M = Manifold;
            static_assert(
                    std::is_base_of_v<
                            RiemannianManifold<
                                    M,
                                    typename M::Domain_t,
                                    M::subspace_dim>,
                            M>,
                    "First template argument must be derived from RiemannianManifold"
            );
        }
    };


}  // namespace dmp
#endif  // DMP_BASE_HPP__
