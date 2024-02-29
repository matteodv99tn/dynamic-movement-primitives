#ifndef DMP_QUATERNION_PERIODIC2_DMP_HPP__
#define DMP_QUATERNION_PERIODIC2_DMP_HPP__

#include <cstddef>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "dmp/basis_function/basis_function.hpp"
#include "dmp/dmp_base.hpp"

namespace dmp {

    class QuaternionPeriodicDmp2 : public DmpBase{
    public:
        QuaternionPeriodicDmp2(
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
                const Eigen::Vector3d&    omega,
                const Eigen::Vector3d&    alpha
        );

        // void batchLearn(
        //         const Eigen::VectorXd& phi,
        //         const Eigen::MatrixXd& q,
        //         const Eigen::MatrixXd& omega,
        //         const Eigen::MatrixXd& alpha
        // );

        double          timeToPhase(const double& t) const;
        Eigen::VectorXd timeToPhase(const Eigen::VectorXd& t) const;

        void setInitialConditions(
                const Eigen::Quaterniond& q0,
                const Eigen::Vector3d&    omega0,
                const double&             phi0 = 0
        );

        void   step();
        double getPhase() const;

        Eigen::Quaterniond getQuaternionState() const;
        Eigen::Vector3d    getAngularVelocityState() const;
        Eigen::Vector3d    getAngularAcceleration() const;

    private:

        Eigen::MatrixXd              _w;
        std::vector<Eigen::MatrixXd> _P;

        // Internal state for integration
        Eigen::Quaterniond  _q, _g;
        // Eigen::Vector3d _y;
        Eigen::Vector3d _omega;
        Eigen::Vector3d _dz_dt;
        double          _phi;

        inline std::size_t _N() const { return _basis->N(); };

        inline double _Omega() const { return 1.0 / _tau; };
    };
}  // namespace dmp


#endif  // DMP_QUATERNION_PERIODIC2_DMP_HPP__
