#ifndef DMPLIB_SECOND_ORDER_SYSTEM_HPP__
#define DMPLIB_SECOND_ORDER_SYSTEM_HPP__

#include <Eigen/Core>
#include <Eigen/src/Core/Matrix.h>

namespace dmp {
template <typename Manifold>
class SecondOrderSystem {
public:
    using Goal_t = typename Manifold::Domain_t;
    using Vector = typename Manifold::Tangent_t;

    Manifold man;
    Goal_t   g;

    SecondOrderSystem();

    void set_goal(const Goal_t& goal);

private:
    double alpha;
    double beta;
};
}  // namespace dmp

#include "dmp/systems/second_order_system.hxx"

#endif  // DMPLIB_SECOND_ORDER_SYSTEM_HPP__
