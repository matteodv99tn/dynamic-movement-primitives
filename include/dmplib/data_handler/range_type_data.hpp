#ifndef DMPLIB_RANGE_TYPE_DATA_HPP
#define DMPLIB_RANGE_TYPE_DATA_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "dmplib/data_handler/concepts.hpp"
#include "dmplib/manifolds/se3_manifold.hpp"

namespace dmp::ranges::internal {

template <typename T>
struct serialised_dimension;

template <::dmp::ranges::concepts::numeric T>
struct serialised_dimension<T> {
    static constexpr int value = 1;
};

template <>
struct serialised_dimension<Eigen::Quaterniond> {
    static constexpr int value = 4;
};

template <int N>
struct serialised_dimension<Eigen::Matrix<double, N, 1>> {
    static constexpr int value = N;
};

template<>
struct serialised_dimension<::dmp::riemannmanifold::SE3>{
    static constexpr int value = 7;
};


}  // namespace dmp::ranges::internal


#endif  // DMPLIB_RANGE_TYPE_DATA_HPP
