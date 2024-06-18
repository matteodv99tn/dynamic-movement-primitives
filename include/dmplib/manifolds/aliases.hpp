#ifndef DMPLIB_RIEMANN_MANIFOLD_ALIASES_HPP
#define DMPLIB_RIEMANN_MANIFOLD_ALIASES_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tuple>
#include <vector>

#include "dmplib/manifolds/riemann_manifold.hpp"

namespace dmp::riemannmanifold {

template <int N>
using Vec_t = Eigen::Matrix<double, N, 1>;

using Vec2_t       = Vec_t<2>;  // NOLINT: are not magic numbers
using Vec3_t       = Vec_t<3>;  // NOLINT
using Vec6_t       = Vec_t<6>;  // NOLINT
using Quaternion_t = Eigen::Quaterniond;

}  // namespace dmp::riemannmanifold

namespace dmp {

template <typename T>
using PosVelSample_t = std::tuple<T, riemannmanifold::tangent_space_t<T>>;

template <typename T>
using PosVelAccSample_t = std::tuple<
        T,
        riemannmanifold::tangent_space_t<T>,
        riemannmanifold::tangent_space_t<T>>;

template <typename T>
using StampedPosSample_t = std::tuple<double, T>;

template <typename T>
using StampedPosVelSample_t =
        std::tuple<double, T, riemannmanifold::tangent_space_t<T>>;

template <typename T>
using StampedPosVelAccSample_t = std::tuple<
        double,
        T,
        riemannmanifold::tangent_space_t<T>,
        riemannmanifold::tangent_space_t<T>>;

template <typename T>
using PosTrajectory_t = std::vector<T>;

template <typename T>
using PosVelTrajectory_t = std::vector<PosVelSample_t<T>>;

template <typename T>
using PosVelAccTrajectory_t = std::vector<PosVelAccSample_t<T>>;

template <typename T>
using StampedPosTrajectory_t = std::vector<StampedPosSample_t<T>>;

template <typename T>
using StampedPosVelTrajectory_t = std::vector<StampedPosVelSample_t<T>>;

template <typename T>
using StampedPosVelAccTrajectory_t = std::vector<StampedPosVelAccSample_t<T>>;

}  // namespace dmp


#endif  // DMPLIB_RIEMANN_MANIFOLD_ALIASES_HPP
