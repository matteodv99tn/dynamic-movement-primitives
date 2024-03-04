#ifndef DMP_QUATERNION_PERIODIC_DMP_HPP__
#define DMP_QUATERNION_PERIODIC_DMP_HPP__

#include <cstddef>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "dmp/basis_function/basis_function.hpp"
#include "dmp/dmp_base.hpp"

namespace dmp {

    class QuaternionPeriodicDmp : public DmpBase {
    public:
        QuaternionPeriodicDmp(
                const BasisFunction::SharedPtr& basis,
                const double                    alpha  = 48.0,
                const double                    lambda = 0.999,
                const double                    dt     = 0.002
        );

        void setObservationPeriod(const double T);

        void resetWeights();

        double          timeToPhase(const double& t) const;
        Eigen::VectorXd timeToPhase(const Eigen::VectorXd& t) const;

        Eigen::Quaterniond getQuaternionState() const;
        Eigen::Vector3d    getAngularVelocityState() const;
        Eigen::Vector3d    getAngularAcceleration() const;
        Eigen::Vector3d    getLogarithm() const;

        Eigen::MatrixXd computeLogarithms(const Eigen::MatrixXd& Qs) const;
        Eigen::MatrixXd getLearnedForcingFunction(const Eigen::VectorXd& phi) const;

        Eigen::VectorXd evaluateDesiredForce(
                const Eigen::Quaterniond& q,
                const Eigen::Vector3d&    omega,
                const Eigen::Vector3d&    domega_dt
        ) const;

        Eigen::MatrixXd evaluateDesiredForce(
                const Eigen::MatrixXd& q,
                const Eigen::MatrixXd& omega,
                const Eigen::MatrixXd& domega_dt
        ) const;

        void setInitialConditions(
                const Eigen::Quaterniond& q0,
                const Eigen::Vector3d&    omega0,
                const double&             phi0 = 0
        );

        void incrementalLearn(
                const double&             phi,
                const Eigen::Quaterniond& q,
                const Eigen::Vector3d&    omega,
                const Eigen::Vector3d&    domega_dt
        );

        void batchLearn(
                const Eigen::VectorXd& phi,
                const Eigen::MatrixXd& q,
                const Eigen::MatrixXd& omega,
                const Eigen::MatrixXd& domega_dt
        );

        void   step();
        void   integration_quat();
        double getPhase() const;


    public:
        // private:
        // double                       _alpha, _beta;
        Eigen::MatrixXd              _w;
        Eigen::MatrixXd              _w_train;
        std::vector<Eigen::MatrixXd> _P;
        Eigen::MatrixXd _fd_des;
        Eigen::Vector3d _curr_log;
        Eigen::MatrixXd _log;

        int idx = 0;
        Eigen::MatrixXd _eta_hist;
        Eigen::MatrixXd _deta_dt_hist;
        Eigen::MatrixXd _q_hist;
        Eigen::VectorXd _phi_hist;
        Eigen::MatrixXd _f_hist;
        Eigen::MatrixXd _log_hist;

        Eigen::Quaterniond _q, _g;
        Eigen::Vector3d    _eta;
        Eigen::Vector3d    _deta_dt;
        double             _phi;

        inline std::size_t _N() const { return _basis->N(); };

        inline double _Omega() const { return 1.0 / _tau; };
    };
}  // namespace dmp


#endif  // DMP_QUATERNION_PERIODIC_DMP_HPP__
