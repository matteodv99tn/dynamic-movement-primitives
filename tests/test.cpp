#include <iostream>
#include <vector>

#include "dmplib/coordinate_systems/periodic_coordinate_system.hpp"
#include "dmplib/manifolds/s3_manifold.hpp"
#include "dmplib/manifolds/se3_manifold.hpp"
#include "dmplib/transformation_systems/second_order_tf.hpp"


using namespace std;

int
main() {
    /*
    dmp::PeriodicCs cs;
    cs.step();

    double                              t = 1;
    dmp::SecondOrderTf<dmp::S3Manifold> tf(std::cref(t));
    std::cout << tf.get_state() << std::endl;

    // // tf.forcing_term_from_demonstration(const PosVelAccSample &demonstration)

    using Vec3 = Eigen::Vector3d;
    using Vec6 = Eigen::Matrix<double, 6, 1>;
    using Quat = Eigen::Quaterniond;

    typename dmp::S3Manifold::PosVelAccTrajectory traj = {
            {Quat::Identity(), Vec3::Zero(), Vec3::Zero()},
            {Quat::Identity(), Vec3::Zero(), Vec3::Zero()},
            {Quat::Identity(), Vec3::Zero(), Vec3::Zero()},
            {Quat::Identity(), Vec3::Zero(), Vec3::Zero()},
            {Quat::Identity(), Vec3::Zero(), Vec3::Zero()},
            {Quat::Identity(), Vec3::Zero(), Vec3::Zero()},
            {Quat::Identity(), Vec3::Zero(), Vec3::Zero()}
    };

    tf.forcing_term_from_demonstration(traj);
    tf.step();

    */

    return 0;
}
