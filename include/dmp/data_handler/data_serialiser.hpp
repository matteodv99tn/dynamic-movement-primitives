#ifndef DMPLIB_DATA_SERIALISER_HPP__
#define DMPLIB_DATA_SERIALISER_HPP__

#include <cstddef>
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace dmp {

template <typename T>
class DataSerialiser {
public:
    static std::size_t              n_entries;
    static std::vector<std::string> labels();
    static std::string              data_extension_name;

    static std::string
    label(const int& idx) {
        return labels()[idx];
    }

    static Eigen::VectorXd serialise_data(const T& entry);
};

}  // namespace dmp

#include "dmp/data_handler/data_serialiser.hxx"


#endif  // DMPLIB_DATA_SERIALISER_HPP__
