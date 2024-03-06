#include "dmp/quaternion.hpp"

using dmp::Quaternion;

Quaternion::Quaternion() {
    _data    = Eigen::Vector4d::Zero();
    _data[0] = 1.0;
}

Quaternion::Quaternion(const double w, const double x, const double y, const double z) {
    _data = Eigen::Vector4d(w, x, y, z);
    _data = _data / _data.norm();
}

Quaternion::Quaternion(const Eigen::Vector4d& data) { _data = data / data.norm(); }

Quaternion::Quaternion(const Eigen::Quaterniond& q) {
    _data[0] = q.w();
    _data[1] = q.x();
    _data[2] = q.y();
    _data[3] = q.z();
}

Quaternion::Quaternion(const Quaternion& other) { _data = other._data; }

Quaternion::Quaternion(const double nu, const Eigen::Vector3d& u) {
    _data[0] = nu;
    _data[1] = u[0];
    _data[2] = u[1];
    _data[3] = u[2];
}

Quaternion Quaternion::operator=(const Quaternion& lhs) {
    if (this != &lhs) { _data = lhs._data; }
    return *this;
}

Quaternion Quaternion::conjugate() const {
    return Quaternion(_data[0], -_data[1], -_data[2], -_data[3]);
}

double Quaternion::norm() const { return _data.norm(); }

Quaternion Quaternion::operator*(const Quaternion& rhs) const {
    const double          nu1 = get_nu();
    const double          nu2 = rhs.get_nu();
    const Eigen::Vector3d u1  = get_u();
    const Eigen::Vector3d u2  = rhs.get_u();

    const double          nu = nu1 * nu2 - u1.dot(u2);
    const Eigen::Vector3d u  = nu1 * u2 + nu2 * u1 + u1.cross(u2);

    return Quaternion(nu, u);
}

Eigen::Vector3d Quaternion::log() const {
    const double          nu = get_nu();
    const Eigen::Vector3d u  = get_u();

    const double acos_nu = std::acos(nu);

    if (u.norm() < 1e-8) return Eigen::Vector3d::Zero();

    return acos_nu * u / u.norm();
}

Eigen::Vector3d dmp::quaternion_log(const Quaternion& q1, const Quaternion& q2) {
    return (q1 * q2.conjugate()).log();
}

Quaternion dmp::quaternion_exp(const Eigen::Vector3d& v) {
    const double v_norm = std::cos(v.norm());
    if (v_norm < 1e-6) return Quaternion();

    const double          cos_vnorm = std::cos(v_norm);
    const double          sin_vnorm = std::sin(v_norm);
    const Eigen::Vector3d u         = sin_vnorm * v / v_norm;
    return Quaternion(cos_vnorm, u);
}

Quaternion dmp::quaternion_exp_map(const Eigen::Vector3d& v, const Quaternion& q) {
    return quaternion_exp(v) * q;
}
