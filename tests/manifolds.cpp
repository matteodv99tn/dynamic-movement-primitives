#include <iostream>

#include "dmplib/class_traits/integrable.hpp"
#include "dmplib/learnable_functions/basis_functions/basis_function.hpp"

int
main() {

    // static_assert(
    //         std::is_default_constructible_v<Eigen::Quaterniond>, "Quat not const"
    // );

    // dmp::S3Manifold  s3;
    // dmp::R3Manifold  r3;
    // dmp::SE3Manifold se3;


    // dmp::Integrable i;

    // dmp::FirstOrderSystem<dmp::S3Manifold> sys;
    // dmp::FirstOrderSystem<dmp::R3Manifold> cartesian;


    // Eigen::Quaterniond q{0.5, 0.5, 0.5, 0.5};
    // q.normalize();
    // sys.set_goal(q);

    // cartesian.set_goal(Eigen::Vector3d{1, 2, 3});

    // std::cout << "Goal: " << sys.g << std::endl;
    // std::cout << "State: " << sys.x << std::endl;

    // dmp::TimeSeriesContainer<Eigen::Quaterniond> data;
    // dmp::TimeSeriesContainer<Eigen::Vector3d>    data_2;

    // sys.start_state_logger();
    // cartesian.start_state_logger();

    // for (int i = 0; i < 10; i++) {
    //     sys.step();
    //     cartesian.step();
    //     data.push_back(sys.x);
    // }

    // auto lg = cartesian.get_state_logger();
    // std::cout << lg->to_table() << std::endl;
    // auto lg2 = sys.get_state_logger();
    // std::cout << lg2->to_table() << std::endl;

    return 0;
}
