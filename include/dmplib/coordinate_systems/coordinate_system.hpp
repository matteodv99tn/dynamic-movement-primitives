#ifndef DMPLIB_COORDINATE_SYSTEM_HPP
#define DMPLIB_COORDINATE_SYSTEM_HPP

#include <cmath>
#include <cstdint>
#include <Eigen/Dense>
#include <functional>
#include <optional>

#include "dmplib/class_traits/integrable.hpp"
#include "dmplib/time_axis.hpp"

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

    CoordinateSystem(TimeAxis* time_axis, const double& initial_value = 1) :
            Integrable<Derived>(time_axis),
            _x(initial_value),
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
        return static_cast<const Derived*>(this)->distribution_on_support_impl(size);
    }

    [[nodiscard]] double
    compute_coordinate(const std::optional<double>& time) const {
        return static_cast<const Derived*>(this)->compute_coord_impl(
                time.value_or(time_axis().get_time())
        );
    }

    [[nodiscard]] std::vector<double>
    compute_coordinate(const std::vector<double>& times) const {
        std::vector<double> coords(times.size());
        for (std::size_t i = 0; i < times.size(); i++)
            coords[i] = compute_coordinate(times[i]);
        return coords;
    }

    [[nodiscard]] Eigen::VectorXd
    compute_coordinate_vec(const std::vector<double>& times) const {
        Eigen::VectorXd coords(times.size());
        for (long i = 0; i < static_cast<long>(times.size()); i++)
            coords(i) = compute_coordinate(times[i]);
        return coords;
    }

    [[nodiscard]] const double* get_coordinate_ptr() const {
        return &_x;
    }

protected:
    using Integrable<Derived>::time_axis;

    [[nodiscard]] double
    T() const {  // NOLINT
        return time_axis().get_period();
    }

    [[nodiscard]] double
    dt() const {
        return time_axis().get_timestep();
    }

    double              _x;          //< coordinate
    Support_t           _support;
};


}  // namespace dmp


#endif  // DMPLIB_COORDINATE_SYSTEM_HPP
