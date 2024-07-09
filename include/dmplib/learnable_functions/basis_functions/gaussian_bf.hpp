#ifndef DMPLIB_GAUSSIAN_BASIS_FUNCTION_HPP
#define DMPLIB_GAUSSIAN_BASIS_FUNCTION_HPP

#include <vector>

#include "dmplib/learnable_functions/basis_functions/basis_function.hpp"

namespace dmp::learnablefunction {

class GaussianBf : public BasisFunction<GaussianBf> {
private:
    std::vector<double> _h;

public:
    GaussianBf(
            const std::size_t& basis_size, const std::vector<double>& function_centers
    );

    void set_h_coefficients(const std::vector<double>& h);

    [[nodiscard]] std::vector<double> get_h_coefficients() const;


protected:
    friend class BasisFunction<GaussianBf>;
    [[nodiscard]] Basis_t evaluate_impl(const double& arg) const;
};


}  // namespace dmp::learnablefunction


#endif  // DMPLIB_GAUSSIAN_BASIS_FUNCTION_HPP
