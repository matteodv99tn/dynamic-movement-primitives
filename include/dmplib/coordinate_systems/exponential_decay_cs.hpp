#ifndef DMPLIB_EXPONENTIAL_DECAY_COORDINATE_SYSTEM_HPP
#define DMPLIB_EXPONENTIAL_DECAY_COORDINATE_SYSTEM_HPP

#include "dmplib/class_traits/integrable.hpp"
#include "dmplib/coordinate_systems/coordinate_system.hpp"

namespace dmp {

class ExponentialDecayCs : public CoordinateSystem<ExponentialDecayCs, POINT_TO_POINT> {
protected:
    double _alpha;

public:
    ExponentialDecayCs(const double& alpha);

    [[nodiscard]] inline double
    get_alpha() const {
        return _alpha;
    }

    inline void
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
};

}  // namespace dmp


#endif  // DMPLIB_EXPONENTIAL_DECAY_COORDINATE_SYSTEM_HPP
