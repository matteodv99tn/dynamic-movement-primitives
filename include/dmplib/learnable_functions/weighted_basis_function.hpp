#ifndef DMPLIB_WEIGHTED_BASIS_FUNCTION_HPP
#define DMPLIB_WEIGHTED_BASIS_FUNCTION_HPP

#include "dmplib/learnable_functions/learnable_function.hpp"

namespace dmp {
    
template <typename Basis, typename Manifold>
class WeightedBasisFunction : LearnableFunction<Manifold>{

public:
    WeightedBasisFunction<Basis, Manifold>(const int& basis_size): _basis(basis_size) {};
    Basis _basis;

};

} // namespace dmp


#endif  // DMPLIB_WEIGHTED_BASIS_FUNCTION_HPP
