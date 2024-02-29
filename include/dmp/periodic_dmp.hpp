#ifndef DMP_PERIODIC_DMP_HPP__
#define DMP_PERIODIC_DMP_HPP__

#include <cstddef>
#include <Eigen/Dense>

#include "dmp/basis_function/basis_function.hpp"
#include "dmp/dmp_base.hpp"

namespace dmp {

    class PeriodicDmp : public DmpBase {
    public:
        PeriodicDmp(
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
                const double& phi, const double& y, const double& dy, const double& ddy
        );
        void incrementalLearn(const double& phi, const Eigen::Vector3d& y_data);


        void batchLearn(
                const Eigen::VectorXd& phi,
                const Eigen::VectorXd& y,
                const Eigen::VectorXd& dy,
                const Eigen::VectorXd& ddy
        );
        void batchLearn(const Eigen::VectorXd& phi, const Eigen::MatrixXd& y_data);

        double          timeToPhase(const double& t) const;
        Eigen::VectorXd timeToPhase(const Eigen::VectorXd& t) const;

        void setInitialConditions(
                const double y0, const double dy0, const double phi0 = 0
        );
        void setInitialConditions(const Eigen::Vector2d& y0, const double phi0 = 0);

        void step();
        Eigen::Vector3d getState() const;
        double          getPhase() const;


    private:

        double _r;

        // Weights for the basis function
        Eigen::VectorXd _w;
        Eigen::MatrixXd _P;

        // Internal state for integration
        double _y;
        double _z;
        double _dz_dt;
        double _phi;

        inline std::size_t _N() const { return _basis->N(); };

        inline double _Omega() const { return 1.0 / _tau; };
    };
}  // namespace dmp


#endif  // DMP_PERIODIC_DMP_HPP__
