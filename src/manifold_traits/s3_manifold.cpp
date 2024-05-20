#include "dmplib/data_handler/data_serialiser.hpp"

using dmp::DataSerialiser;

template <>
std::size_t DataSerialiser<Eigen::Quaterniond>::n_entries = 4;

template <>
std::vector<std::string>
DataSerialiser<Eigen::Quaterniond>::labels() {
    return {"x", "y", "z", "w"};
}

template <>
Eigen::VectorXd
DataSerialiser<Eigen::Quaterniond>::serialise_data(const Eigen::Quaterniond& entry) {
    return entry.coeffs();
}
