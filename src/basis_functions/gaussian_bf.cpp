#include "dmplib/learnable_functions/basis_functions/gaussian_bf.hpp"

#include <cmath>
#include <iostream>

#include "fmt/core.h"
#include "range/v3/view/transform.hpp"
#include "range/v3/view/zip.hpp"
#include "range/v3/view/iota.hpp"
#include "range/v3/view/sliding.hpp"

namespace rs = ranges;
namespace rv = ranges::views;


using Gbf = dmp::GaussianBf;

Gbf::GaussianBf(
        const std::size_t& basis_size,
        const double&      alpha,
        const double&      min_support,
        const double&      max_support,
        const bool&        include_ub
) :
        BasisFunction<GaussianBf>(basis_size, min_support, max_support, include_ub) {
    _alpha = alpha;
};

Gbf::Basis
Gbf::evaluate_impl(const double& arg) {
    Basis res = Basis::Zero(_basis_size);

    rs::copy(
            rv::zip(_c, _h) | rv::transform([x = arg](const auto&& tpl) {
                auto& [c, h] = tpl;
                return std::exp(-h * std::pow(x - c, 2.0));
            }),
            res.data()
    );

    return res;
}

void
Gbf::init_on_support(const bool& /* include_ub */) {
    auto ci_formula = [a = _alpha, n = _basis_size](const int& i) -> double {
        return std::exp((-a * i - 1.0) / (n - 1));
    };

    auto hi_formula = [](const auto&& tpl) -> double {
        auto& [h_curr, h_next] = tpl;
        return 1 / std::pow(h_next - h_curr, 2.0);
    };

    _c.reserve(_basis_size);
    _h.reserve(_basis_size);
    rs::copy(rv::iota(_basis_size) | rv::transform(ci_formula), _c.data());
    rs::copy(_c | rv::sliding(2) | rv::transform(hi_formula), _h.data());
    _h.back() = _h[_basis_size - 2];
}

void
Gbf::set_c_coefficients(const std::vector<double>& c) {
    if (c.size() != _basis_size) {
        std::cerr << fmt::format(
                "DMPLIB WARN: Provided vector for c coefficients has {} elements, but "
                "{} are expected. Leaving content unchanged",
                c.size(),
                _basis_size
        ) << std::endl;
        return;
    }
    _c = c;
}

std::vector<double>
Gbf::get_c_coefficients() const {
    return _c;
}

void
Gbf::set_h_coefficients(const std::vector<double>& h) {
    if (h.size() != _basis_size) {
        std::cerr << fmt::format(
                "DMPLIB WARN: Provided vector for h coefficients has {} elements, but "
                "{} are expected. Leaving content unchanged",
                h.size(),
                _basis_size
        ) << std::endl;
        return;
    }
    _h = h;
}

std::vector<double>
Gbf::get_h_coefficients() const {
    return _h;
}
