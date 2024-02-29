#ifndef DMP_QUATERNION_PERIODIC_DMP_HPP__
#define DMP_QUATERNION_PERIODIC_DMP_HPP__

#include <cstddef>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "dmp/basis_function/basis_function.hpp"
#include "dmp/multidof_periodic_dmp.hpp"

namespace dmp {

    class QuaternionPeriodicDmp {
    public:
        QuaternionPeriodicDmp(
                const BasisFunction::SharedPtr& basis,
                const double                    alpha  = 48.0,
                const double                    lambda = 0.999,
                const double                    dt     = 0.002
        );

        void setObservationPeriod(const double T);
        void setSamplingPeriod(const double dt);
        void setTau(const double tau);

        void resetWeights();

        void incrementalLearn(
                const double&             phi,
                const Eigen::Quaterniond& q,
                const Eigen::VectorXd&    omega,
                const Eigen::VectorXd&    alpha
        );

        void batchLearn(
                const Eigen::VectorXd& phi,
                const Eigen::MatrixXd& q,
                const Eigen::MatrixXd& omega,
                const Eigen::MatrixXd& alpha
        );

        double          timeToPhase(const double& t) const;
        Eigen::VectorXd timeToPhase(const Eigen::VectorXd& t) const;

        void setInitialConditions(
                const Eigen::Quaterniond& q0,
                const Eigen::VectorXd&    omega0,
                const double&             phi0 = 0
        );

        void   step();
        void   integration_quat();
        double getPhase() const;

        Eigen::Quaterniond getQuaternionState() const;
        Eigen::VectorXd    getAngularVelocityState() const;
        Eigen::VectorXd    getAngularAcceleration() const;
        Eigen::Vector3d    getLogarithm() const;


    public:
        // private:
        double              _alpha, _beta;
        MultiDofPeriodicDmp _dmp;
        Eigen::Quaterniond  _q, _g;
        Eigen::MatrixXd q_log;

        inline double _dt() const { return _dmp.getSamplingPeriod(); }

    };
}  // namespace dmp


#endif  // DMP_QUATERNION_PERIODIC_DMP_HPP__
