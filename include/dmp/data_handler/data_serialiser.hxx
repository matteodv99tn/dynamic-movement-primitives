#ifndef DMPLIB_DATA_SERIALISER_HXX__
#define DMPLIB_DATA_SERIALISER_HXX__

#include <Eigen/Dense>

#include "dmp/data_handler/data_serialiser.hpp"
#include "range/v3/all.hpp"

// Declarations of traits that requires templates!


namespace dmp {

template <int N>
class DataSerialiser<Eigen::Vector<double, N>> {
public:
    const static std::size_t n_entries = N;

    std::vector<std::string>
    labels() {
        return ranges::views::iota(1, N + 1)
               | ranges::views::transform([](auto i) -> std::string {
                     return std::string("x") + std::to_string(i);
                 })
               | ranges::to_vector;
    }

    static std::string
    label(const int& idx) {
        return labels()[idx];
    }

    static Eigen::VectorXd
    serialise_data(const Eigen::Vector<double, N>& entry) {
        return entry;
    }
};

};  // namespace dmp

#endif  // DMPLIB_DATA_SERIALISER_HXX__
