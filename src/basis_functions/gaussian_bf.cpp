#include "dmplib/learnable_functions/basis_functions/gaussian_bf.hpp"

#include <cmath>
#include <iostream>

#include "fmt/core.h"
#include "fmt/ostream.h"
#include "range/v3/view/iota.hpp"
#include "range/v3/view/sliding.hpp"
#include "range/v3/view/transform.hpp"
#include "range/v3/view/zip.hpp"

namespace rs = ranges;
namespace rv = ranges::views;


using Gbf_t = dmp::GaussianBf;

Gbf_t::GaussianBf(
        const std::size_t& basis_size,
        const double&      alpha,
        const double&      min_support,
        const double&      max_support,
        const bool&        include_ub
) :
        BasisFunction<GaussianBf>(basis_size, min_support, max_support, include_ub),
        _alpha(alpha){};

Gbf_t::Basis_t
Gbf_t::evaluate_impl(const double& arg) {
    Basis_t res = Basis_t::Zero(static_cast<Eigen::Index>(_basis_size));

    rs::copy(
            rv::zip(_c, _h) | rv::transform([x = arg](const auto&& tpl) {
                auto& [c, h] = tpl;
                return std::exp(-h * std::pow(x - c, 2.0));  // NOLINT
            }),
            res.data()
    );

    return res;
}

void
Gbf_t::init_on_support(const bool& /* include_ub */) {
    auto ci_formula = [a = _alpha, n = static_cast<double>(_basis_size)](const int& i
                      ) -> double { return std::exp((-a * i - 1.0) / (n - 1.0)); };

    auto hi_formula = [](const auto&& tpl) -> double {
        auto& [h_curr, h_next] = tpl;
        return 1 / std::pow(h_next - h_curr, 2.0);  // NOLINT
    };

    _c.reserve(_basis_size);
    _h.reserve(_basis_size);
    rs::copy(rv::iota(_basis_size) | rv::transform(ci_formula), _c.data());
    rs::copy(_c | rv::sliding(2) | rv::transform(hi_formula), _h.data());
    _h.back() = _h[_basis_size - 2];
}

void
Gbf_t::set_c_coefficients(const std::vector<double>& c) {
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
Gbf_t::get_c_coefficients() const {
    return _c;
}

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
