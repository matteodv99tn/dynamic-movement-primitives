#include "dmplib/dmp.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <memory>

#include "dmplib/coordinate_systems/exponential_decay_cs.hpp"
#include "dmplib/learnable_functions/basis_functions/gaussian_bf.hpp"
#include "dmplib/learnable_functions/weighted_basis_function.hpp"
#include "dmplib/manifolds/aliases.hpp"
#include "dmplib/manifolds/se3_manifold.hpp"
#include "dmplib/transformation_systems/second_order_tf.hpp"


using dmp::ExponentialDecayCs;
using dmp::learnablefunction::GaussianBf;
using dmp::learnablefunction::WeightedBasisFunction;
using dmp::riemannmanifold::SE3;
using dmp::transformationsystem::SecondOrderTs;

using Func_t = WeightedBasisFunction<GaussianBf, SE3>;

using Dmp_t = dmp::Dmp<SE3, ExponentialDecayCs, SecondOrderTs<SE3>, Func_t>;

int
main() {
    Dmp_t dmp;

    dmp.set_period(2.0);
    dmp.initialise_coordinate_system(dmp.get_period());
    dmp.initialise_transformation_system(dmp.get_period());

    const std::size_t         n_basis = 25;  // NOLINT
    const std::vector<double> c = dmp.coord_sys().distribution_on_support(n_basis);
    const GaussianBf          b(n_basis, c);
    dmp.initialise_learnable_function(GaussianBf(n_basis, c));

    dmp::StampedPosVelAccTrajectory_t<SE3> traj;
    using Sample_t = dmp::StampedPosVelAccSample_t<SE3>;
    using Vec_t    = Eigen::Vector<double, 6>;  // NOLINT

    Sample_t sample = {0, SE3(), Vec_t::Zero(), Vec_t::Zero()};
    traj.push_back(sample);

    for (std::size_t i = 0; i < 250; ++i) {
        std::get<0>(sample) = i * 100000;
        traj.push_back(sample);
    }

    dmp.batch_learn(traj, true);

    return 0;
}
