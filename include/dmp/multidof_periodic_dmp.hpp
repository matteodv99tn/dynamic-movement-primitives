#ifndef DMP_MULTIDOF_PERIODIC_DMP_HPP__
#define DMP_MULTIDOF_PERIODIC_DMP_HPP__

#include <cstddef>
#include <Eigen/Dense>
#include <memory>

#include "dmp/basis_function/basis_function.hpp"
#include "dmp/dmp_base.hpp"

namespace dmp {

    class MultiDofPeriodicDmp : public DmpBase {
    public:
        using UniquePtr  = std::unique_ptr<MultiDofPeriodicDmp>;
        using SharedPtr  = std::shared_ptr<MultiDofPeriodicDmp>;

        MultiDofPeriodicDmp(
                const BasisFunction::SharedPtr& basis,
                const std::size_t               n_dof,
                const double                    alpha  = 48.0,
                const double                    lambda = 0.999,
                const double                    dt     = 0.002
        );

        void setObservationPeriod(const double T);

        void resetWeights();

        void incrementalLearn(
                const double&          phi,
                const Eigen::VectorXd& y,
                const Eigen::VectorXd& dy,
                const Eigen::VectorXd& ddy
        );

        void batchLearn(
                const Eigen::VectorXd& phi,
                const Eigen::MatrixXd& y,
                const Eigen::MatrixXd& dy,
                const Eigen::MatrixXd& ddy
        );

        double          timeToPhase(const double& t) const;
        Eigen::VectorXd timeToPhase(const Eigen::VectorXd& t) const;

        void setInitialConditions(
                const Eigen::VectorXd& y0,
                const Eigen::VectorXd& dy0,
                const double&          phi0 = 0
        );

        void   step();
        double getPhase() const;

        void            setPositionState(const Eigen::VectorXd& y);
        Eigen::VectorXd getPositionState() const;
        Eigen::VectorXd getVelocityState() const;
        Eigen::VectorXd getAccelerationState() const;
        Eigen::VectorXd getZ() const;
        double          getOmega() const;

        Eigen::VectorXd evaluateDesiredForce(
                const Eigen::VectorXd& y,
                const Eigen::VectorXd& dy,
                const Eigen::VectorXd& ddy
        ) const;

        Eigen::MatrixXd evaluateDesiredForce(
                const Eigen::MatrixXd& y,
                const Eigen::MatrixXd& dy,
                const Eigen::MatrixXd& ddy
        ) const;

        Eigen::MatrixXd getLearnedForcingFunction(const Eigen::VectorXd& phi) const;

    private:
        std::size_t _n_dof;

        // Weights for the basis function
        Eigen::VectorXd              _r;
        Eigen::MatrixXd              _w;
        std::vector<Eigen::MatrixXd> _P;

        // Internal state for integration
        Eigen::VectorXd _y;
        Eigen::VectorXd _z;
        Eigen::VectorXd _dz_dt;
        double          _phi;

        inline std::size_t _N() const { return _basis->N(); };

        inline double _Omega() const { return 1.0 / _tau; };
    };
}  // namespace dmp


#endif  // DMP_MULTIDOF_PERIODIC_DMP_HPP__
