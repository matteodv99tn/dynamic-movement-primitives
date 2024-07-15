#include "dmplib/learnable_functions/basis_functions/gaussian_bf.hpp"

#include <cmath>
#include <iostream>

#include "range/v3/view/sliding.hpp"
#include "range/v3/view/take.hpp"
#include "range/v3/view/transform.hpp"

namespace rs = ranges;
namespace rv = ranges::views;


using Gbf_t = dmp::learnablefunction::GaussianBf;

Gbf_t::GaussianBf(
        const std::size_t& basis_size, const std::vector<double>& function_centers
) :
        BasisFunction<GaussianBf>(basis_size, function_centers), _h(_basis_size) {
    std::cout << "Gaussian Basis function constructor" << std::endl;


    for(std::size_t i{0}; i < _basis_size - 1; ++i) {
        const double diff  = _c[i+1] - _c[i];
        _h[i] = 1 / (diff * diff);
    }
    _h.back() = _h[_basis_size - 2];

    std::cout << "Provided centers: " << std::endl << "   ";
    for (double c : _c) std::cout << c << " ";
    std::cout << std::endl;

    std::cout << "Computed widths: " << std::endl << "   ";
    for (double h : _h) std::cout << h << " ";
    std::cout << std::endl;
};

void
Gbf_t::set_h_coefficients(const std::vector<double>& h) {
    assert(h.size() == _basis_size);
    _h = h;
}

std::vector<double>
Gbf_t::get_h_coefficients() const {
    return _h;
}

Gbf_t::Basis_t
Gbf_t::evaluate_impl(const double& arg) const {
    Eigen::VectorXd res(_basis_size);
    for (std::size_t i = 0; i < _basis_size; i++) {
        const double diff = arg - _c[i];
        res(i)            = std::exp(-_h[i] * diff * diff);  // NOLINT
    }
    return res;
}
