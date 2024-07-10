#ifndef DMPLIB_EXPONENTIAL_DECAY_COORDINATE_SYSTEM_HPP
#define DMPLIB_EXPONENTIAL_DECAY_COORDINATE_SYSTEM_HPP

#include "dmplib/class_traits/integrable.hpp"
#include "dmplib/coordinate_systems/coordinate_system.hpp"
#include "dmplib/time_axis.hpp"

namespace dmp {

class ExponentialDecayCs : public CoordinateSystem<ExponentialDecayCs, POINT_TO_POINT> {
protected:
    double _alpha;

public:
    ExponentialDecayCs(
            TimeAxis::Reference time_axis,
            const double&       alpha = 1.0 / 5.0  // NOLINT
    );

    [[nodiscard]] double
    get_alpha() const {
        return _alpha;
    }

    void
    set_alpha(const double& alpha) {
        _alpha = alpha;
    }

protected:
    // CRTP traits definition
    friend class Integrable<ExponentialDecayCs>;
    void step_impl();

    friend class CoordinateSystem<ExponentialDecayCs, POINT_TO_POINT>;
    [[nodiscard]] std::vector<double> distribution_on_support_impl(
            const std::size_t& size
    ) const;

    [[nodiscard]] double compute_coord_impl(const double& time) const;
};

}  // namespace dmp


#endif  // DMPLIB_EXPONENTIAL_DECAY_COORDINATE_SYSTEM_HPP
