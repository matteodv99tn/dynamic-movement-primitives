#ifndef DMP_DUMMY_MANIFOLD_HPP__
#define DMP_DUMMY_MANIFOLD_HPP__

#include <cmath>
#include <Eigen/Geometry>

#include "dmp/manifolds/riemann_manifolds.hpp"

namespace dmp {

struct Test {
    int a, b;

    Test() = delete;

    Test(int x, int y) {
        a = x;
        b = y;
    }
};

// Space of a quaternion
class DummyManifold : public RiemannManifold<DummyManifold, Test, 3> {
public:
#if 1
    static inline Test
    construct_domain_impl() {
        return Test(1, 2);
    }
#endif

    Tangent_t
    logarithmic_map_impl(const Test& application_point, const Test& y) const {
        return Tangent_t::Zero();
    }

    Domain_t
    exponential_map_impl(const Test& application_point, const Tangent_t& v) const {
        return Test(1, 2);
    }
};

}  // namespace dmp

#endif  // DMP_DUMMY_MANIFOLD_HPP__
