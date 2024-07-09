#include "dmplib/learnable_functions/basis_functions/radial_gaussian_bf.hpp"

#include <cmath>
#include <iostream>
#include <Eigen/Dense>

#include "fmt/ostream.h"
#include "range/v3/view/transform.hpp"
#include "range/v3/view/zip.hpp"

namespace rs = ranges;
namespace rv = ranges::views;


using Rgbf_t = dmp::learnablefunction::RadialGaussianBf;

Rgbf_t::RadialGaussianBf(
        const std::size_t& basis_size, const std::vector<double>& function_centers
) :
        BasisFunction<RadialGaussianBf>(basis_size, function_centers) {
    _h = std::vector(_basis_size, 2.5 / static_cast<double>(_basis_size));  // NOLINT
};

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
