#ifndef DMPLIB_FIRST_ORDER_SYSTEM_HPP__
#define DMPLIB_FIRST_ORDER_SYSTEM_HPP__

#include <Eigen/Core>
#include <memory>

#include "dmp/class_traits/integrable.hpp"
#include "dmp/data_handler/timeseries.hpp"

namespace dmp {
template <typename Manifold>
class FirstOrderSystem : public Integrable {
public:
    using Goal_t = typename Manifold::Domain_t;
    using Vector = typename Manifold::Tangent_t;

    template <typename S>
    using LoggerPtr = std::unique_ptr<TimeSeriesContainer<S>>;

    Manifold man;
    Goal_t   g;  // Goal configuration
    Goal_t   x;  // Current configuration

    FirstOrderSystem();

    void set_goal(const Goal_t& goal);

    void step();

    void start_goal_logger();

    void start_state_logger();

    // void assign_goal_logger(const LoggerPtr<Goal_t>& logger);
    // void assign_state_logger(const LoggerPtr<Goal_t>& logger);

    LoggerPtr<Goal_t> get_goal_logger();

    LoggerPtr<Goal_t> get_state_logger();

private:
    double            _alpha;
    LoggerPtr<Goal_t> _goal_logger;
    LoggerPtr<Goal_t> _state_logger;
};
}  // namespace dmp

#include "dmp/systems/first_order_system.hxx"

#endif  // DMPLIB_FIRST_ORDER_SYSTEM_HPP__
