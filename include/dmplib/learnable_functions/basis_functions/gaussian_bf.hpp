#ifndef DMPLIB_GAUSSIAN_BASIS_FUNCTION_HPP__
#define DMPLIB_GAUSSIAN_BASIS_FUNCTION_HPP__

#include <vector>

#include "dmplib/learnable_functions/basis_functions/basis_function.hpp"

namespace dmp {

class GaussianBf : public BasisFunction<GaussianBf> {
private:
    std::vector<double> _c;
    std::vector<double> _h;

public:
    GaussianBf(
            const std::size_t& basis_size,
            const double&      alpha       = 48.0,
            const double&      min_support = 0.0,
            const double&      max_support = 1.0,
            const bool&        include_ub  = true
    );

    void                set_c_coefficients(const std::vector<double>& c);
    std::vector<double> get_c_coefficients() const;

    void                set_h_coefficients(const std::vector<double>& h);
    std::vector<double> get_h_coefficients() const;


protected:
    friend class BasisFunction<GaussianBf>;
    Basis evaluate_impl(const double& arg);

    void init_on_support(const bool& include_ub);


private:
    double _alpha;
};


}  // namespace dmp


#endif  // DMPLIB_GAUSSIAN_BASIS_FUNCTION_HPP__
