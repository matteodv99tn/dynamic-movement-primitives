#ifndef DMPLIB_COORDINATE_SYSTEM_HPP
#define DMPLIB_COORDINATE_SYSTEM_HPP

#include <cmath>
#include <functional>

#include "dmplib/class_traits/integrable.hpp"

namespace dmp {
enum RepresentationType {
    POINT_TO_POINT,
    PERIODIC
};

template <typename Derived, RepresentationType Rep_Type = POINT_TO_POINT>
class CoordinateSystem : public Integrable<Derived> {
public:
    using Support_t                          = std::tuple<double, double>;
    static constexpr RepresentationType type = Rep_Type;

protected:
    double _x;  //< coordinate
    double _T;  //< observation period NOLINT

    Support_t _support;

public:
    CoordinateSystem(
            const double& initial_value = 1, const double& observation_period = 1
    ) :
            _x(initial_value),
            _T(observation_period),
            _support(
                    (Rep_Type == POINT_TO_POINT) ? std::make_tuple(0.0, 1.0)
                                                 : std::make_tuple(0.0, 2 * M_PI)
            ) {}

    [[nodiscard]] inline double
    get_coordinate() const {
        return _x;
    }

    inline void
    set_coordinate(const double& x) {
        _x = x;
    }

    inline double
    get_observation_period() {
        return _T;
    }

    inline void
    set_observation_period(const double& T) {
        _T = T;
    }

    inline std::reference_wrapper<const double>
    get_observation_period_reference() {
        return std::cref(_T);
    }

    [[nodiscard]] inline Support_t
    get_coordinate_support() const {
        return _support;
    }

    /**
     * @brief Creates a vector of coordinate value that are "equally distributed" in the
     * time domain.
     *
     */
    [[nodiscard]] std::vector<double>
    distribution_on_support(const std::size_t& size) const {
        return static_cast<Derived*>(this)->distribution_on_support_impl(size);
    }
};

}  // namespace dmp


#endif  // DMPLIB_COORDINATE_SYSTEM_HPP
