#ifndef DMP__QUATERNION_HPP__
#define DMP__QUATERNION_HPP__

#include <Eigen/Dense>

namespace dmp {
    class Quaternion {
    public:
        Quaternion();
        Quaternion(const double w, const double x, const double y, const double z);
        Quaternion(const Eigen::Vector4d& data);
        Quaternion(const Eigen::Quaterniond& q);
        Quaternion(const double nu, const Eigen::Vector3d& u);
        Quaternion(const Quaternion& other);
        Quaternion operator=(const Quaternion& lhs);

        /**
         * @brief Returns a the conjugate of the quaternion.
         */
        Quaternion conjugate() const;

        /**
         * @brief Computes the L2 norm of the quaternion.
         */
        double norm() const;

        /**
         * @brief Multiplication in the sense of quaternions.
         */
        Quaternion operator*(const Quaternion& rhs) const;

        /**
         * @brief Performs the logarithmic mapping.
         */
        Eigen::Vector3d log() const;


    private:
        Eigen::Vector4d _data;

        inline double get_nu() const { return _data[0]; };

        inline Eigen::Vector3d get_u() const { return _data.tail(3); };
    };

    /**
     * @brief Exponential mapping operator.
     *
     * Takes a 3D vector and builds the corresponding quaternion
     */
    Quaternion quaternion_exp(const Eigen::Vector3d& v);

    /**
     * @brief Logarithmic mapping operator.
     *
     * Takes two quaternions and returns the logarithm of their product.
     * In particular it computes Log(q1 * q2.conjugate())
     */
    Eigen::Vector3d quaternion_log(const Quaternion& q1, const Quaternion& q2);

    /**
     * @brief Calls the exponential mapping operator to "rotate" the quaternion.
     *
     * Build a quaternion that lies on the geodesic starting from q and pointing in the
     * direction of v.
     */
    Quaternion quaternion_exp_map(const Eigen::Vector3d& v, const Quaternion& q);

}  // namespace dmp


#endif  // DMP__QUATERNION_HPP__
