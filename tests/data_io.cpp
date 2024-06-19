/**
 * Example file that shows the capabilities for exporting and importing data
 */
#include <cstdio>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

#include "dmplib/data_handler/conversions.hpp"
#include "dmplib/manifolds/aliases.hpp"
#include "range/v3/range/conversion.hpp"
#include "range/v3/view/enumerate.hpp"
#include "range/v3/view/zip.hpp"
#include "fmt/format.h"
#include "fmt/ostream.h"

namespace rs = ranges;
namespace rv = ranges::views;

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

    for (std::size_t i = 0; i < n_samples; ++i) {
        quaternions[i]   = Quat_t::UnitRandom();
        velocities[i]    = Vec3_t::Random();
        accelerations[i] = Vec3_t::Random();
    }

    // --- Simple serialisations
    const std::string q1_as_str = dmp::to::string(quaternions[0]);
    fmt::print("First quaternion (as string): {}", q1_as_str);

    const std::string v1_as_str = dmp::to::string(velocities[0]);
    fmt::print("First velocity (as string): {}", v1_as_str);

    fmt::print("Doubling second element of the vector");
    std::vector<double> v1_as_vec = dmp::to::vector(velocities[0]);
    v1_as_vec[1] *= 2;

    fmt::print("Reconstructing Vec3_t from the string");
    auto v1_reconstruct = dmp::from::vector<Vec3_t>(v1_as_vec);
    fmt::print("Resulinting vector: {}", dmp::to::string(v1_reconstruct));
    // To properly reconstruct the type, it is always necessary to specify the return
    // type


    // --- Tuples
    using Tpl_t = std::tuple<Quat_t, Vec3_t, Vec3_t>;
    const Tpl_t tpl{quaternions[0], velocities[0], accelerations[0]};

    fmt::print(
            "The library is ment to work a lot with tuples to aggregate types, e.g."
    );
    fmt::print("Quaternion:   {}", dmp::to::string(quaternions[0]));
    fmt::print("Velocity:     {}", dmp::to::string(velocities[0]));
    fmt::print("Acceleration: {}", dmp::to::string(accelerations[0]));
    fmt::print(" -> Tuple:    {}", dmp::to::string(tpl));

    // --- Trajectory
    using Trajectory_t = std::vector<Tpl_t>;
    const Trajectory_t traj =
            rv::zip(quaternions, velocities, accelerations) | rs::to<Trajectory_t>;

    dmp::to::file("data_io_test.csv", traj);
    fmt::print("Written trajectory to file 'data_io_test.csv'; content:");
    const auto lines = dmp::to::string(traj);
    for (const auto& line : lines) fmt::print("{}", line);

    fmt::print("Retrieving trajectory from the same file");
    const Trajectory_t loaded_traj = dmp::from::file<Tpl_t>("data_io_test.csv");
    // Parses each line of the file as of type "Tpl_t", thus creating a
    // std::vector<Tpl_t> = Trajectory_t

    fmt::print("Loaded data:");
    const auto retrieved_lines = dmp::to::string(loaded_traj);
    for (const auto& line : retrieved_lines) fmt::print("{}", line);


    // --- TimeStamped trajectory
    using StampedTraj_t        = dmp::StampedPosTrajectory_t<Eigen::Quaterniond>;
    const StampedTraj_t traj2  = rv::enumerate(quaternions) | rs::to<StampedTraj_t>;
    const auto          lines2 = dmp::to::string(traj2);
    fmt::print("Stamped trajectory:");
    for (const auto& line : lines2) fmt::print("{}", line);

    fmt::print("Writing stamped trajectory to 'data_io_test2.csv'");
    dmp::to::file("data_io_test2.csv", traj2);

    fmt::print("Loading stamped trajectory from 'data_io_test2.csv'");
    const auto loaded_traj2 = dmp::from::file<dmp::StampedPosSample_t<Eigen::Quaterniond>>("data_io_test2.csv");

    fmt::print("Loaded data:");
    const auto retrieved_lines2 = dmp::to::string(loaded_traj);
    for (const auto& line : retrieved_lines2) fmt::print("{}", line);

}
