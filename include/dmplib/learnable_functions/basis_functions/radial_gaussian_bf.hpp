#ifndef DMPLIB_RADIAL_GAUSSIAN_BASIS_FUNCTION_HPP
#define DMPLIB_RADIAL_GAUSSIAN_BASIS_FUNCTION_HPP

#include <vector>

#include "dmplib/learnable_functions/basis_functions/basis_function.hpp"

namespace dmp::learnablefunction {

class RadialGaussianBf : public BasisFunction<RadialGaussianBf> {
private:
    using Bf = BasisFunction<RadialGaussianBf>;  // NOLINT: type case
    using Bf::_c;
    std::vector<double> _h;

public:
    RadialGaussianBf(
            const std::size_t& basis_size, const std::vector<double>& function_centers

    );

    void set_h_coefficients(const std::vector<double>& h);

    [[nodiscard]] std::vector<double> get_h_coefficients() const;


protected:
    friend class BasisFunction<RadialGaussianBf>;
    Basis_t evaluate_impl(const double& arg);
};


}  // namespace dmp


#endif  // DMPLIB_RADIAL_GAUSSIAN_BASIS_FUNCTION_HPP
