#ifndef DMPLIB_NUMERIC_DIFFERENTIATION_HPP
#define DMPLIB_NUMERIC_DIFFERENTIATION_HPP

#include "dmplib/manifolds/aliases.hpp"
#include "dmplib/manifolds/concepts.hpp"

namespace dmp::utils {

template <dmp::riemannmanifold::concepts::riemann_manifold T>
StampedPosVelTrajectory_t<T>
differentiate(const StampedPosTrajectory_t<T>& traj) {
    using dmp::riemannmanifold::logarithmic_map;

    StampedPosVelTrajectory_t<T> out(traj.size());
    for (std::size_t i = 0; i < traj.size(); ++i) {
        std::get<0>(out[i]) = std::get<0>(traj[i]);
        std::get<1>(out[i]) = std::get<1>(traj[i]);

        if (i == 0) {
            std::get<2>(out[i]) = dmp::riemannmanifold::tangent_space_t<T>::Zero();
        } else {
            const T&     curr_pt = std::get<1>(out[i]);
            const T&     prev_pt = std::get<1>(out[i - 1]);
            const double dt = (std::get<0>(out[i]) - std::get<0>(out[i - 1])) * 1e-9;
            std::get<2>(out[i]) = logarithmic_map(prev_pt, curr_pt) / dt;
        }
    }
    return out;
}

template <dmp::riemannmanifold::concepts::riemann_manifold T>
StampedPosVelAccTrajectory_t<T>
differentiate_twice(const StampedPosTrajectory_t<T>& traj) {
    using dmp::riemannmanifold::logarithmic_map;

    StampedPosVelAccTrajectory_t<T> out(traj.size());
    for (std::size_t i = 0; i < traj.size(); ++i) {
        std::get<0>(out[i]) = std::get<0>(traj[i]);
        std::get<1>(out[i]) = std::get<1>(traj[i]);

        if (i == 0) {
            std::get<2>(out[i]) = dmp::riemannmanifold::tangent_space_t<T>::Zero();
            std::get<3>(out[i]) = dmp::riemannmanifold::tangent_space_t<T>::Zero();
        } else {
            const T&     curr_pt  = std::get<1>(out[i]);
            const T&     prev_pt  = std::get<1>(out[i - 1]);
            const auto&  prev_vel = std::get<2>(out[i - 1]);
            const double dt = (std::get<0>(out[i]) - std::get<0>(out[i - 1])) * 1e-9;
            const auto&  curr_vel = logarithmic_map(prev_pt, curr_pt) / dt;
            std::get<2>(out[i])   = curr_vel;
            std::get<3>(out[i])   = (curr_vel - prev_vel) / dt;
        }
    }
    return out;
}

}  // namespace dmp::utils


#endif  // DMPLIB_NUMERIC_DIFFERENTIATION_HPP
