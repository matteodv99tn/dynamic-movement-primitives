#include "dmplib/coordinate_systems/exponential_decay_cs.hpp"

#include <cmath>

#include "dmplib/coordinate_systems/coordinate_system.hpp"
#include "dmplib/time_axis.hpp"
#include "range/v3/algorithm/copy.hpp"
#include "range/v3/view/iota.hpp"
#include "range/v3/view/take.hpp"
#include "range/v3/view/transform.hpp"

namespace rs = ranges;
namespace rv = ranges::views;

using Edcs_t   = dmp::ExponentialDecayCs;
using TimeAxis = dmp::TimeAxis;  // NOLINT

Edcs_t::ExponentialDecayCs(TimeAxis* time_axis, const double& alpha) :
        CoordinateSystem(time_axis, 1), _alpha(alpha) {
}

void
Edcs_t::step_impl() {
    _x -= _alpha * _x * dt() / T();
}

std::vector<double>
Edcs_t::distribution_on_support_impl(const std::size_t& size) const {
    auto ci_formula = [a = _alpha, n = static_cast<double>(size)](const int& i
                      ) -> double { return std::exp((-a * i - 1.0) / (n - 1.0)); };

    std::vector<double> c(size);
    rs::copy(rv::iota(0) | rv::take(size) | rv::transform(ci_formula), c.data());

    return c;
}

double
Edcs_t::compute_coord_impl(const double& time) const {
    return std::exp(-_alpha / T() * time);
}
