#ifndef DMPLIB_FORCING_FUNCTION_HPP__
#define DMPLIB_FORCING_FUNCTION_HPP__

#include <Eigen/Dense>

namespace dmp {

template <int N, bool LEARNABLE = false>
class ForcingFunction {
public:
    using Output                   = Eigen::Vector<double, N>;
    const static bool is_learnable = LEARNABLE;

    Output evaluate(double);
};

template <int N>
class LearnableForcingFunction : public ForcingFunction<N, true> {};


}  // namespace dmp


#endif  // DMPLIB_FORCING_FUNCTION_HPP__
