/**
 * Example file that shows the capabilities for exporting and importing data
 */
#include <cstdio>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <type_traits>
#include <vector>

#include "dmplib/manifolds/se3_manifold.hpp"
#include "fmt/format.h"
#include "fmt/ostream.h"
#include "range/v3/algorithm/copy.hpp"
#include "range/v3/range/conversion.hpp"
#include "range/v3/view/enumerate.hpp"
#include "range/v3/view/zip.hpp"

// clang-format off
#include "dmplib/data_handler/conversions.hpp"
#include "dmplib/data_handler/range_serialise.hpp"
#include "dmplib/data_handler/concepts.hpp"
#include "dmplib/manifolds/aliases.hpp"
// clang-format on

namespace rs = ranges;
namespace rv = ranges::views;

template <typename T>
struct tmp_t : std::false_type {};

using Quat_t = Eigen::Quaterniond;
using Vec3_t = Eigen::Vector3d;

int
main() {
    // Create a fake trajectory of quaternions and their respective
    // velocities/acceleration
    const std::size_t   n_samples = 10;
    std::vector<Quat_t> quaternions(n_samples);
    std::vector<Vec3_t> velocities(n_samples);
    std::vector<Vec3_t> accelerations(n_samples);
    static_assert(dmp::ranges::concepts::eigen_vector<Vec3_t>);
    // static_assert(ranges::forward_iterator<dmp::ranges::StrVec_t>);

    for (std::size_t i = 0; i < n_samples; ++i) {
        quaternions[i]   = Quat_t::UnitRandom();
        velocities[i]    = Vec3_t::Random();
        accelerations[i] = Vec3_t::Random();
    }

    auto res   = dmp::ranges::serialise(quaternions[0]);
    auto res_b = dmp::ranges::serialise(velocities[0]);
    auto res_c = dmp::ranges::serialise(quaternions);
    auto res_d = dmp::ranges::serialise(velocities);

    static_assert(dmp::ranges::concepts::eigen_quaternion<Quat_t>);


    // --- Simple serialisations
    const std::string q1_as_str = dmp::to::string(quaternions[0]);
    fmt::println("First quaternion (as string): {}", q1_as_str);

    const std::string v1_as_str = dmp::to::string(velocities[0]);
    fmt::println("First velocity (as string): {}", v1_as_str);

    fmt::print("Reconstructing Vec3_t from the string");
    auto v1_reconstruct = dmp::from::string<Vec3_t>(v1_as_str);
    fmt::println("Resulinting vector: {}", dmp::to::string(v1_reconstruct));
    // To properly reconstruct the type, it is always necessary to specify the return
    // type


    // --- Tuples
    using Tpl_t = std::tuple<Quat_t, Vec3_t, Vec3_t>;
    const Tpl_t tpl{quaternions[0], velocities[0], accelerations[0]};

    fmt::println(
            "The library is ment to work a lot with tuples to aggregate types, e.g."
    );
    fmt::println("Quaternion:    {}", dmp::to::string(quaternions[0]));
    fmt::println("Velocity:      {}", dmp::to::string(velocities[0]));
    fmt::println("Acceleration:  {}", dmp::to::string(accelerations[0]));

    const std::string tpl_str = dmp::to::string(tpl);
    fmt::println(" -> Tuple:     {}", tpl_str);
    auto tpl_reconstructed = dmp::from::string<Tpl_t>(tpl_str);
    fmt::println("Reconstructed: {}", dmp::to::string(tpl_reconstructed));

    // --- Trajectory
    using Trajectory_t = std::vector<Tpl_t>;
    const Trajectory_t traj =
            rv::zip(quaternions, velocities, accelerations) | rs::to<Trajectory_t>;
    // auto res3 = dmp::ranges::serialise(traj);

    dmp::to::file("data_io_test.csv", traj);
    fmt::println("Written trajectory to file 'data_io_test.csv'; content:");
    const auto lines = dmp::to::string(traj);
    for (const auto& line : lines) fmt::println("{}", line);

    fmt::println("Retrieving trajectory from the same file");
    const Trajectory_t loaded_traj = dmp::from::file<Tpl_t>("data_io_test.csv");
    // Parses each line of the file as of type "Tpl_t", thus creating a
    // std::vector<Tpl_t> = Trajectory_t

    fmt::println("Trajectory size: {}", loaded_traj.size());

    fmt::println("Loaded data:");
    const auto retrieved_lines = dmp::to::string(loaded_traj);
    for (const auto& line : retrieved_lines) fmt::println("{}", line);


    // --- TimeStamped trajectory
    using StampedTraj_t        = dmp::StampedPosTrajectory_t<Eigen::Quaterniond>;
    const StampedTraj_t traj2  = rv::enumerate(quaternions) | rs::to<StampedTraj_t>;
    const auto          lines2 = dmp::to::string(traj2);
    fmt::println("Stamped trajectory:");
    for (const auto& line : lines2) fmt::println("{}", line);

    fmt::println("Writing stamped trajectory to 'data_io_test2.csv'");
    dmp::to::file("data_io_test2.csv", traj2);

    fmt::println("Loading stamped trajectory from 'data_io_test2.csv'");
    const auto loaded_traj2 =
            dmp::from::file<dmp::StampedPosSample_t<Eigen::Quaterniond>>(
                    "data_io_test2.csv"
            );

    fmt::println("Loaded data:");
    const auto retrieved_lines2 = dmp::to::string(loaded_traj2);
    for (const auto& line : retrieved_lines2) fmt::println("{}", line);

    // --- Pose trajectories
    using PoseTraj_t = dmp::PosTrajectory_t<dmp::riemannmanifold::SE3>;

    PoseTraj_t traj3;
    for (std::size_t i = 0; i < n_samples; i++)
        traj3.emplace_back(velocities[i], quaternions[i]);

    fmt::println("Writing stamped trajectory to 'data_io_test3.csv'");
    dmp::to::file("data_io_test3.csv", traj3);

    fmt::println("Loading stamped trajectory from 'data_io_test3.csv'");
    const auto loaded_traj3 =
            dmp::from::file<dmp::riemannmanifold::SE3>("data_io_test3.csv");
}
