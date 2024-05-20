#ifndef DMPLIB_FIRST_ORDER_SYSTEM_HXX__
#define DMPLIB_FIRST_ORDER_SYSTEM_HXX__

#include <memory>

#include "dmplib/data_handler/timeseries.hpp"
#include "dmplib/systems/first_order_system.hpp"

namespace dmp {

template <typename M>
FirstOrderSystem<M>::FirstOrderSystem() {
    _alpha = 5.0;

    g = man.construct_domain();
    x = man.construct_domain();

    _goal_logger  = nullptr;
    _state_logger = nullptr;
}

template <typename M>
void
FirstOrderSystem<M>::set_goal(const Goal_t& goal) {
    g = goal;
}

template <typename M>
void
FirstOrderSystem<M>::step() {
    Vector dv = man.logarithmic_map(x, g);
    x         = man.exponential_map(x, dv * _alpha, _dt);

    // Logging
    if (_state_logger) _state_logger->push_back(x);
    if (_goal_logger) _goal_logger->push_back(g);
}

template <typename M>
void
FirstOrderSystem<M>::start_goal_logger() {
    _goal_logger = std::make_unique<TimeSeriesContainer<Goal_t>>(this);
}

template <typename M>
void
FirstOrderSystem<M>::start_state_logger() {
    _state_logger = std::make_unique<TimeSeriesContainer<Goal_t>>(this);
}

// template <typename M>
// void FirstOrderSystem<M>::assign_goal_logger(const LoggerPtr<Goal_t>& logger) {
//     _goal_logger = std::move(logger);
// }
//
// template <typename M>
// void FirstOrderSystem<M>::assign_state_logger(const LoggerPtr<Goal_t>& logger) {
//     _state_logger = std::move(logger);
// }

template <typename M>
FirstOrderSystem<M>::LoggerPtr<typename FirstOrderSystem<M>::Goal_t>
FirstOrderSystem<M>::get_goal_logger() {
    return std::move(_goal_logger);
}

template <typename M>
FirstOrderSystem<M>::LoggerPtr<typename FirstOrderSystem<M>::Goal_t>
FirstOrderSystem<M>::get_state_logger() {
    return std::move(_state_logger);
}

}  // namespace dmp


#endif  // DMPLIB_FIRST_ORDER_SYSTEM_HXX__
