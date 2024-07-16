#ifndef DMPLIB_DMP_HPP
#define DMPLIB_DMP_HPP

// #include <functional>
// #include <iostream>
#include <memory>
#include <stdexcept>

// #include "dmplib/data_handler/conversions.hpp"
#include "dmplib/manifolds/aliases.hpp"
// #include "dmplib/manifolds/se3_manifold.hpp"
#include "dmplib/time_axis.hpp"
#include "transformation_systems/modified_second_order_tf.hpp"

namespace dmp {

template <
        typename Manifold,
        typename Coordinate_System,
        typename Transformation_System,
        typename Learnable_Function>
class Dmp {
public:
    using CoordinateSystem_t     = Coordinate_System;
    using TransformationSystem_t = Transformation_System;
    using LearnableFunction_t    = Learnable_Function;

    using StampedSample_t     = dmp::StampedPosVelAccSample_t<Manifold>;
    using StampedTrajectory_t = dmp::StampedPosVelAccTrajectory_t<Manifold>;

    static constexpr double ts_to_s = 1e-9;

    Dmp() : _cs(nullptr), _ts(nullptr), _fun(nullptr) {};

    void
    batch_learn(
            const dmp::StampedPosVelAccTrajectory_t<Manifold>& traj,
            bool apply_distance_scaling = true
    ) {
        assert(ready());
        dmp::TimeStamp_t ti_uint = std::get<0>(traj.front());
        dmp::TimeStamp_t tf_uint = std::get<0>(traj.back());

        std::vector<double> times(traj.size());
        for (std::size_t i = 0; i < traj.size(); ++i)
            times[i] = (std::get<0>(traj[i]) - ti_uint) * ts_to_s;

        time_axis().set_posix_period(tf_uint - ti_uint);  // NOLINT

        transf_sys().set_initial_pos_state(std::get<1>(traj.front()));
        transf_sys().set_pos_goal_state(std::get<1>(traj.back()));

        if (apply_distance_scaling) transf_sys().enable_force_scaling();
        else transf_sys().disable_force_scaling();

        const Eigen::VectorXd s_coords = coord_sys().compute_coordinate_vec(times);

        Eigen::MatrixXd f_des;
        if constexpr (std::is_same_v<
                              TransformationSystem_t,
                              dmp::transformationsystem::ModifiedSecondOrderTs<
                                      Manifold>>) {
            f_des = transf_sys().evaluate_forcing_term_matrix(traj, s_coords);
        } else {
            f_des = transf_sys().evaluate_forcing_term_matrix(traj);
        }
        for (long i = 0; i < traj.size(); ++i) { f_des.row(i) /= s_coords(i); }
        learnable_func().learn(s_coords, f_des);
    }

    StampedTrajectory_t
    integrate_trajectory(
            const StampedSample_t& start_pos,
            const StampedSample_t& goal,
            const double&          T,
            const double&          dt,
            const double&          traj_T
    ) {
        time_axis().reset_time();
        time_axis().set_period(T);
        time_axis().set_integration_timestep(dt);
        coord_sys().set_coordinate(1.0);
        transf_sys().set_initial_pos_state(std::get<Manifold>(start_pos));
        transf_sys().set_pos_state(std::get<Manifold>(start_pos));
        transf_sys().set_pos_goal_state(std::get<Manifold>(goal));
        transf_sys().reset_velocity_state();

        StampedTrajectory_t traj;

        while (_time_axis.get_time() < traj_T) {
            traj.emplace_back(
                    time_axis().time_as_timestamp(),
                    transf_sys().get_pos_state(),
                    transf_sys()._z,
                    transf_sys()._dz_dt
            );
            const double s = coord_sys().get_coordinate();
            coord_sys().step();
            const auto f = learnable_func().evaluate(s);

            transf_sys().set_forcing_term(f * s);
            transf_sys().step();
            time_axis().step();
        }
        return traj;
    }

    CoordinateSystem_t&
    coord_sys() {
        if (!_cs) { throw std::runtime_error("No coordinate system set"); }
        return *_cs;
    }

    TransformationSystem_t&
    transf_sys() {
        if (!_ts) { throw std::runtime_error("No transformation system set"); }
        return *_ts;
    }

    LearnableFunction_t&
    learnable_func() {
        if (!_fun) { throw std::runtime_error("No learnable function set"); }
        return *_fun;
    }

    template <typename... Args>
    void
    initialise_coordinate_system(Args... args) {
        _cs = std::make_unique<CoordinateSystem_t>(args...);
    }

    template <typename... Args>
    void
    initialise_transformation_system(Args... args) {
        _ts = std::make_unique<TransformationSystem_t>(args...);
    }

    template <typename... Args>
    void
    initialise_learnable_function(Args... args) {
        _fun = std::make_unique<LearnableFunction_t>(args...);
    }

    [[nodiscard]] bool
    ready() const {
        return _cs && _ts && _fun;
    }

    double
    get_period() {
        return _time_axis.get_period();
    }

    void
    set_period(const double& T) {
        _time_axis.set_period(T);
    }

    [[nodiscard]] TimeAxis&
    time_axis() {
        return _time_axis;
    }

    [[nodiscard]] TimeAxis*
    time_axis_ptr() {
        return &_time_axis;
    }


private:
    std::unique_ptr<CoordinateSystem_t>     _cs;
    std::unique_ptr<TransformationSystem_t> _ts;
    std::unique_ptr<LearnableFunction_t>    _fun;

    TimeAxis _time_axis;
};
}  // namespace dmp


#endif  // DMPLIB_DMP_HPP
