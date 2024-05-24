#ifndef DMPLIB_DOMAIN_HPP
#define DMPLIB_DOMAIN_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "dmplib/manifolds/domains/se3.hpp"

namespace dmp{

template <typename T>
struct tangent_space_dimension;

template <typename T>
struct tangent_space_t {
    using Tangent_t = Eigen::Matrix<double, tangent_space_dimension<T>::value, 1>;
};


template <>
struct tangent_space_dimension<double> {
    static const int value = 1;
};

template <int N>
struct tangent_space_dimension<Eigen::Matrix<double, N, 1>> {
    static const int value = N;
};

template <>
struct tangent_space_dimension<Eigen::Quaterniond> {
    static const int value = 3;
};

template <>
struct tangent_space_dimension<SE3> {
    static const int value = 6;
};

}


#endif  // DMPLIB_DOMAIN_HPP
