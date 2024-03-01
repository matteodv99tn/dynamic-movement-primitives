#define CATCH_CONFIG_MAIN
#include "catch2/catch.hpp"
// #include "catch2/catch_test_macros.hpp"
// #include "catch2/catch_config.hpp"


#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "dmp/quaternion_utils.hpp"

#define IS_ZERO(x, y) (std::abs(x - y) < 1e-6)

TEST_CASE("Logarithmic map on identity", "[quaternion]") {
    Eigen::Quaterniond q     = Eigen::Quaterniond::Identity();
    Eigen::Vector3d    log_q = dmp::logarithmic_map(q);
    REQUIRE(log_q.norm() == 0.0);
}

TEST_CASE("Logarithmic map 1", "[quaternion]") {
    Eigen::Quaterniond q(-0.5, 0.5, 0.5, 0.5);
    Eigen::Vector3d    log_q = dmp::logarithmic_map(q);
    REQUIRE(IS_ZERO(log_q[0], 1.209199576156145));
    REQUIRE(IS_ZERO(log_q[1], 1.209199576156145));
    REQUIRE(IS_ZERO(log_q[2], 1.209199576156145));
}

TEST_CASE("Logarithmic map 2", "[quaternion]") {
    Eigen::Quaterniond q(
            -0.7106690545187014,
            -0.5685352436149612,
            0.2132007163556104,
            0.3553345272593507
    );
    Eigen::Vector3d log_q = dmp::logarithmic_map(q);
    REQUIRE(IS_ZERO(log_q[0], -1.908174181612675));
    REQUIRE(IS_ZERO(log_q[1], 0.7155653181047529));
    REQUIRE(IS_ZERO(log_q[2], 1.192608863507921));
}

TEST_CASE("Exponential map on identity", "[quaternion]") {
    Eigen::Vector3d    omega = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q     = dmp::exponential_map(omega);
    REQUIRE(IS_ZERO(q.w(), 1.0));
    REQUIRE(q.vec().norm() < 1e-6);
}

TEST_CASE("Exponential map 1", "[quaternion]") {
    Eigen::Vector3d    omega = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q     = dmp::exponential_map(omega);
    REQUIRE(IS_ZERO(q.w(), 1.0));
    REQUIRE(q.vec().norm() < 1e-6);
}

TEST_CASE("Exponential map 2", "[quaternion]") {
    Eigen::Vector3d    omega(1.0, 1.0, 1.0);
    Eigen::Quaterniond q = dmp::exponential_map(omega);
    REQUIRE(IS_ZERO(q.x(), 0.569860099182514));
    REQUIRE(IS_ZERO(q.y(), 0.569860099182514));
    REQUIRE(IS_ZERO(q.z(), 0.569860099182514));
    REQUIRE(IS_ZERO(q.w(), -0.1605565385746905));
}

TEST_CASE("Exponential map 3", "[quaternion]") {
    Eigen::Vector3d    omega(1.0, 2.0, 3.0);
    Eigen::Quaterniond q = dmp::exponential_map(omega);
    REQUIRE(IS_ZERO(q.x(), -0.1509213272199645));
    REQUIRE(IS_ZERO(q.y(), -0.301842654439929));
    REQUIRE(IS_ZERO(q.z(), -0.4527639816598935));
    REQUIRE(IS_ZERO(q.w(), -0.8252990620752587));
}
