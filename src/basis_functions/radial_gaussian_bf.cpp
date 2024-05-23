#include "dmplib/learnable_functions/basis_functions/radial_gaussian_bf.hpp"

#include <cmath>
#include <Eigen/src/Core/util/Meta.h>
#include <iostream>
#include <ostream>

#include "fmt/ostream.h"
#include "range/v3/view/transform.hpp"
#include "range/v3/view/zip.hpp"

namespace rs = ranges;
namespace rv = ranges::views;


using Rgbf_t = dmp::RadialGaussianBf;

Rgbf_t::RadialGaussianBf(
        const std::size_t& basis_size,
        const double&      min_support,
        const double&      max_support,
        const bool&        include_ub
) :
        BasisFunction<RadialGaussianBf>(
                basis_size, min_support, max_support, include_ub
        ){};

Rgbf_t::Basis_t
Rgbf_t::evaluate_impl(const double& arg) {
    Basis_t res = Basis_t::Zero(static_cast<Eigen::Index>(_basis_size));

    rs::copy(
            rv::zip(_c, _h) | rv::transform([x = arg](const auto&& tpl) {
                auto& [c, h] = tpl;
                return std::exp(h * (std::cos(x - c) - 1.0));
            }),
            res.data()
    );

    return res;
}

void
Rgbf_t::init_on_support(const bool& include_ub) {
    const double support_delta = _support.second - _support.first;
    const double c_start       = _support.first;
    double       c_end         = _support.second;

    const auto nbasis_double = static_cast<double>(_basis_size);
    const auto nbasis_index  = static_cast<Eigen::Index>(_basis_size);


    if (!include_ub) c_end -= support_delta / nbasis_double;

    _c.reserve(_basis_size);
    rs::copy(Eigen::VectorXd::LinSpaced(nbasis_index, c_start, c_end), _c.data());
    _h = std::vector(_basis_size, 2.5 / nbasis_double);  // NOLINT
}

void
Rgbf_t::set_c_coefficients(const std::vector<double>& c) {
    if (c.size() != _basis_size) {
        fmt::println(
                std::cerr,
                "DMPLIB WARN: Provided vector for c coefficients has {} elements, but "
                "{} are expected. Leaving content unchanged",
                c.size(),
                _basis_size
        );
        return;
    }
    _c = c;
}

std::vector<double>
Rgbf_t::get_c_coefficients() const {
    return _c;
}

void
Rgbf_t::set_h_coefficients(const std::vector<double>& h) {
    if (h.size() != _basis_size) {
        fmt::println(
                std::cerr,
                "DMPLIB WARN: Provided vector for h coefficients has {} elements, but "
                "{} are expected. Leaving content unchanged",
                h.size(),
                _basis_size
        );
        return;
    }
    _h = h;
}

std::vector<double>
Rgbf_t::get_h_coefficients() const {
    return _h;
}
