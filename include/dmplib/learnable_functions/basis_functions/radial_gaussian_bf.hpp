#ifndef DMPLIB_RADIAL_GAUSSIAN_BASIS_FUNCTION_HPP__
#define DMPLIB_RADIAL_GAUSSIAN_BASIS_FUNCTION_HPP__

#include <vector>

#include "dmplib/learnable_functions/basis_functions/basis_function.hpp"

namespace dmp {

class RadialGaussianBf : public BasisFunction<RadialGaussianBf> {
private:
    std::vector<double> _c;
    std::vector<double> _h;

public:
    RadialGaussianBf(
            const std::size_t& basis_size,
            const double&      min_support = 0.0,
            const double&      max_support = M_PI,
            const bool&        include_ub  = false
    );

    void                set_c_coefficients(const std::vector<double>& c);
    std::vector<double> get_c_coefficients() const;

    void                set_h_coefficients(const std::vector<double>& h);
    std::vector<double> get_h_coefficients() const;


protected:
    friend class BasisFunction<RadialGaussianBf>;
    Basis evaluate_impl(const double& arg);

    void init_on_support(const bool& include_ub);

};


}  // namespace dmp


#endif  // DMPLIB_RADIAL_GAUSSIAN_BASIS_FUNCTION_HPP__
