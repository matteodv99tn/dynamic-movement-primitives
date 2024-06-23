#ifndef DMPLIB_SERIALISATION_CONCEPTS_HPP
#define DMPLIB_SERIALISATION_CONCEPTS_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>
#include <type_traits>

namespace dmp::ranges::concepts {

namespace helpers {

    template <typename T>
    struct is_eigen_vector : std::false_type {};

    template <int N>
    struct is_eigen_vector<Eigen::Matrix<double, N, 1>> : std::true_type {
        static constexpr int dim = N;
    };

}  // namespace helpers

template <typename T>
concept eigen_quaternion = std::is_same_v<T, Eigen::Quaterniond>;


template <typename T>
concept eigen_vector = helpers::is_eigen_vector<T>::value;

template <typename T>
concept stringable = requires(T obj) {
    { std::to_string(obj) };
};

template <typename T>
concept numeric = std::integral<T> || std::floating_point<T>;


}  // namespace dmp::ranges::concepts


#endif  // DMPLIB_SERIALISATION_CONCEPTS_HPP
