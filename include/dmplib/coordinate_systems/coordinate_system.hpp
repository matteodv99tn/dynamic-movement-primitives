#ifndef DMPLIB_COORDINATE_SYSTEM_HPP
#define DMPLIB_COORDINATE_SYSTEM_HPP

#include <cmath>
#include <cstdint>
#include <Eigen/Dense>
#include <functional>

#include "dmplib/class_traits/integrable.hpp"

namespace dmp {
enum RepresentationType : std::uint8_t {
    POINT_TO_POINT,
    PERIODIC
};

template <typename Derived, RepresentationType Rep_Type = POINT_TO_POINT>
class CoordinateSystem : public Integrable<Derived> {
public:
    using Support_t                          = std::tuple<double, double>;
    static constexpr RepresentationType type = Rep_Type;

protected:
    double                               _x;  //< coordinate
    std::reference_wrapper<const double> _T;  //< observation period NOLINT

    Support_t _support;

public:
    CoordinateSystem(
            std::reference_wrapper<const double>& observation_period,
            const double&                         initial_value = 1
    ) :
            _x(initial_value),
            _T(observation_period),
            _support(
                    (Rep_Type == POINT_TO_POINT) ? std::make_tuple(0.0, 1.0)
                                                 : std::make_tuple(0.0, 2 * M_PI)
            ) {}

    [[nodiscard]] double
    get_coordinate() const {
        return _x;
    }

    void
    set_coordinate(const double& x) {
        _x = x;
    }

    double
    get_observation_period() {
        return _T;
    }

    void
    set_observation_period(const std::reference_wrapper<const double> T) {
        _T = T;
    }

    std::reference_wrapper<const double>
    get_observation_period_reference() {
        return std::cref(_T);
    }

    [[nodiscard]] Support_t
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
        return static_cast<const Derived* const>(this)->distribution_on_support_impl(
                size
        );
    }

    [[nodiscard]] double
    compute_coordinate(const double& time) const {
        return static_cast<const Derived*>(this)->compute_coord_impl(time);
    }

    [[nodiscard]]
    std::vector<double>
    compute_coordinate(const std::vector<double>& times) const {
        std::vector<double> coords(times.size());
        for (std::size_t i = 0; i < times.size(); i++)
            coords[i] = compute_coordinate(times[i]);
        return coords;
    }

    [[nodiscard]]
    Eigen::VectorXd
    compute_coordinate_vec(const std::vector<double>& times) const {
        Eigen::VectorXd coords(times.size());
        for (long i = 0; i < times.size(); i++)
            coords(i) = compute_coordinate(times[i]);
        return coords;
    }
};

}  // namespace dmp


#endif  // DMPLIB_COORDINATE_SYSTEM_HPP
