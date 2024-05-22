#ifndef DMPLIB_BASIS_FUNCTION_HXX__
#define DMPLIB_BASIS_FUNCTION_HXX__

#include "dmplib/learnable_functions/basis_functions/basis_function.hpp"
#include "range/v3/algorithm/copy.hpp"
#include "range/v3/view/take.hpp"
#include "range/v3/view/transform.hpp"

namespace dmp {

template <typename D>
BasisFunction<D>::BasisFunction(
        const std::size_t& basis_size,
        const double&      min_support,
        const double&      max_support,
        const bool&        include_ub
) :
        _basis_size(basis_size), _support({min_support, max_support}) {
    static_cast<D*>(this)->init_on_support(include_ub);
}

template <typename D>
typename BasisFunction<D>::Basis
BasisFunction<D>::evaluate(const double& arg, const bool& normalize) const {
    Basis out = static_cast<const D*>(this)->evaluate_impl(arg);
    if (normalize) out /= out.norm();
    return out;
}

template <typename D>
double
BasisFunction<D>::evaluate(
        const double& arg, const Weights& weights, const bool& normalize
) const {
    Basis base = evaluate(arg, normalize);
    return base.transpose() * weights;
}

template <typename D>
template <int N>
Eigen::Matrix<double, N, 1>
BasisFunction<D>::evaluate(
        const double& arg, const WeightsVector& weights, const bool& normalize
) const {
    namespace rs = ranges;
    namespace rv = ranges::views;

    Eigen::Matrix<double, N, 1> res;
    std::size_t                 n_elems = N;
    if (N == Eigen::Dynamic) {
        res     = Eigen::VectorXd(weights.size());
        n_elems = weights.size();
    }


    rs::copy(
            weights | rv::transform([this, arg, normalize](const Weights& w) {
                return evaluate(arg, w, normalize);
            }) | rv::take(n_elems),
            res.data()
    );
    return res;
}


}  // namespace dmp

#endif  // DMPLIB_BASIS_FUNCTION_HXX__
