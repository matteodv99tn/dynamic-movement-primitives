#include "dmplib/learnable_functions/basis_functions/gaussian_bf.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <iostream>

#include "fmt/core.h"
#include "fmt/ostream.h"
#include "range/v3/view/sliding.hpp"
#include "range/v3/view/transform.hpp"

namespace rs = ranges;
namespace rv = ranges::views;


using Gbf_t = dmp::learnablefunction::GaussianBf;

Gbf_t::GaussianBf(
        const std::size_t& basis_size, const std::vector<double>& function_centers
) :
        BasisFunction<GaussianBf>(basis_size, function_centers), _h(_basis_size) {
    auto hi_formula = [](const auto&& tpl) -> double {
        auto& [h_curr, h_next] = tpl;
        return 1 / std::pow(h_next - h_curr, 2.0);  // NOLINT
    };
    rs::copy(_c | rv::sliding(2) | rv::transform(hi_formula), _h.data());
    _h.back() = _h[_basis_size - 2];
};

void
Gbf_t::set_h_coefficients(const std::vector<double>& h) {
    if (h.size() != _basis_size) {
        fmt::print(
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
Gbf_t::get_h_coefficients() const {
    return _h;
}

Gbf_t::Basis_t
Gbf_t::evaluate_impl(const double& arg) const {
    Eigen::VectorXd res;
    for(std::size_t i = 0; i < _basis_size; i++){
        res(i) =  std::exp(_h[i] * std::pow(arg - _c[i], 2.0));
    }
    return res;
}
