#ifndef DMPLIB_DATA_TRAITS_HPP
#define DMPLIB_DATA_TRAITS_HPP

#include <Eigen/Dense>
#include <tuple>
#include <type_traits>

namespace dmp::type_traits {


template <typename T>
struct is_eigen_vector : std::false_type {};

template <int N>
struct is_eigen_vector<Eigen::Matrix<double, N, 1>> : std::true_type {};

template <typename T>
using is_eigen_vector_t = typename is_eigen_vector<T>::value;  // NOLINT

template <typename T>
struct is_std_vector : std::false_type {};

template <typename T>
struct is_std_vector<std::vector<T>> : std::true_type {
    using type = T; // NOLINT
};

template <typename T>
using is_std_vector_t = typename is_std_vector<T>::value;  // NOLINT

template <typename T>
using contained_type = is_std_vector<T>::type; // NOLINT

template <typename T>
struct is_tuple : std::false_type {};

template <typename T>
struct is_tuple<std::tuple<T>> : std::true_type {};

}  // namespace dmp::type_traits


#endif  // DMPLIB_DATA_TRAITS_HPP
