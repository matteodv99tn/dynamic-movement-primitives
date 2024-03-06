#define CATCH_CONFIG_MAIN
#include "catch2/catch.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "dmp/quaternion_utils.hpp"

#define IS_ZERO(x, y) (std::abs(x - y) < 1e-3)


TEST_CASE("Matlab test 1", "[quaternion]") {
	Eigen::Quaterniond q1(0.6940241097, 0.6244115353, 0.0967577711, 0.3450778223);
	Eigen::Quaterniond q2(0.7534754748, 0.5857654913, 0.0641896121, 0.2916045127);
	Eigen::Vector3d v(0.8341890611, 0.0156446927, 0.8637108651);
	std::cout << "q1: " << q1 << std::endl;
	std::cout << "q2: " << q2 << std::endl;
	std::cout << "q1 (matlab): 0.6940241097, 0.6244115353, 0.0967577711, 0.3450778223"  << std::endl;
	std::cout << "q2 (matlab): 0.7534754748, 0.5857654913, 0.0641896121, 0.2916045127"  << std::endl;

	Eigen::Vector3d log = dmp::logarithmic_map(q1, q2);
	std::cout << "log (computed): " << log.transpose() << std::endl;
	std::cout << "log (matlab):  0.0579652852, 0.0083144138, 0.0743346344" << std::endl;
	REQUIRE(IS_ZERO(log(0), 0.0579652852));
	REQUIRE(IS_ZERO(log(1), 0.0083144138));
	REQUIRE(IS_ZERO(log(2), 0.0743346344));

	Eigen::Quaterniond exp = dmp::exponential_map(v, q1);
	std::cout << "exponential (computed): " << exp << std::endl;
	std::cout << "exponential (matlab):  -0.3860688781, 0.6145477200, 0.2386367940, 0.6452398017" << std::endl;
	REQUIRE(IS_ZERO(exp.w(), -0.3860688781));
	REQUIRE(IS_ZERO(exp.x(), 0.6145477200));
	REQUIRE(IS_ZERO(exp.y(), 0.2386367940));
	REQUIRE(IS_ZERO(exp.z(), 0.6452398017));

}


TEST_CASE("Matlab test 2", "[quaternion]") {
	Eigen::Quaterniond q1(0.0900596361, 0.7718004748, 0.5770385074, 0.2514753468);
	Eigen::Quaterniond q2(0.5326543560, 0.1138607295, 0.6254194937, 0.5587177536);
	Eigen::Vector3d v(0.0559761574, 0.0563430185, 0.1525006370);
	std::cout << "q1: " << q1 << std::endl;
	std::cout << "q2: " << q2 << std::endl;
	std::cout << "q1 (matlab): 0.0900596361, 0.7718004748, 0.5770385074, 0.2514753468"  << std::endl;
	std::cout << "q2 (matlab): 0.5326543560, 0.1138607295, 0.6254194937, 0.5587177536"  << std::endl;

	Eigen::Vector3d log = dmp::logarithmic_map(q1, q2);
	std::cout << "log (computed): " << log.transpose() << std::endl;
	std::cout << "log (matlab):  0.2691312626, 0.7462533745, -0.3806098277" << std::endl;
	REQUIRE(IS_ZERO(log(0), 0.2691312626));
	REQUIRE(IS_ZERO(log(1), 0.7462533745));
	REQUIRE(IS_ZERO(log(2), -0.3806098277));

	Eigen::Quaterniond exp = dmp::exponential_map(v, q1);
	std::cout << "exponential (computed): " << exp << std::endl;
	std::cout << "exponential (matlab):  -0.0247718192, 0.6919694608, 0.6766924970, 0.2503035891" << std::endl;
	REQUIRE(IS_ZERO(exp.w(), -0.0247718192));
	REQUIRE(IS_ZERO(exp.x(), 0.6919694608));
	REQUIRE(IS_ZERO(exp.y(), 0.6766924970));
	REQUIRE(IS_ZERO(exp.z(), 0.2503035891));

}


TEST_CASE("Matlab test 3", "[quaternion]") {
	Eigen::Quaterniond q1(0.0174554292, 0.3871438813, 0.7403666296, 0.5492469180);
	Eigen::Quaterniond q2(0.3823130857, 0.6349729822, 0.0718113444, 0.6674497338);
	Eigen::Vector3d v(0.1080166941, 0.5169967581, 0.1431560221);
	std::cout << "q1: " << q1 << std::endl;
	std::cout << "q2: " << q2 << std::endl;
	std::cout << "q1 (matlab): 0.0174554292, 0.3871438813, 0.7403666296, 0.5492469180"  << std::endl;
	std::cout << "q2 (matlab): 0.3823130857, 0.6349729822, 0.0718113444, 0.6674497338"  << std::endl;

	Eigen::Vector3d log = dmp::logarithmic_map(q1, q2);
	std::cout << "log (computed): " << log.transpose() << std::endl;
	std::cout << "log (matlab):  -0.3578062284, 0.2155474755, 0.7213178952" << std::endl;
	REQUIRE(IS_ZERO(log(0), -0.3578062284));
	REQUIRE(IS_ZERO(log(1), 0.2155474755));
	REQUIRE(IS_ZERO(log(2), 0.7213178952));

	Eigen::Quaterniond exp = dmp::exponential_map(v, q1);
	std::cout << "exponential (computed): " << exp << std::endl;
	std::cout << "exponential (matlab):  -0.4635657219, 0.5016250565, 0.6371219512, 0.3571480693" << std::endl;
	REQUIRE(IS_ZERO(exp.w(), -0.4635657219));
	REQUIRE(IS_ZERO(exp.x(), 0.5016250565));
	REQUIRE(IS_ZERO(exp.y(), 0.6371219512));
	REQUIRE(IS_ZERO(exp.z(), 0.3571480693));

}


TEST_CASE("Matlab test 4", "[quaternion]") {
	Eigen::Quaterniond q1(0.4393440531, 0.0035969546, 0.6021717862, 0.6665980834);
	Eigen::Quaterniond q2(0.6262147085, 0.6741270851, 0.3450201128, 0.1853886018);
	Eigen::Vector3d v(0.1007505119, 0.5078488308, 0.5856091257);
	std::cout << "q1: " << q1 << std::endl;
	std::cout << "q2: " << q2 << std::endl;
	std::cout << "q1 (matlab): 0.4393440531, 0.0035969546, 0.6021717862, 0.6665980834"  << std::endl;
	std::cout << "q2 (matlab): 0.6262147085, 0.6741270851, 0.3450201128, 0.1853886018"  << std::endl;

	Eigen::Vector3d log = dmp::logarithmic_map(q1, q2);
	std::cout << "log (computed): " << log.transpose() << std::endl;
	std::cout << "log (matlab):  -0.2027640309, -0.2577738961, 0.8554210213" << std::endl;
	REQUIRE(IS_ZERO(log(0), -0.2027640309));
	REQUIRE(IS_ZERO(log(1), -0.2577738961));
	REQUIRE(IS_ZERO(log(2), 0.8554210213));

	Eigen::Quaterniond exp = dmp::exponential_map(v, q1);
	std::cout << "exponential (computed): " << exp << std::endl;
	std::cout << "exponential (matlab):  -0.3159243349, 0.0297322047, 0.5698415335, 0.7580161194" << std::endl;
	REQUIRE(IS_ZERO(exp.w(), -0.3159243349));
	REQUIRE(IS_ZERO(exp.x(), 0.0297322047));
	REQUIRE(IS_ZERO(exp.y(), 0.5698415335));
	REQUIRE(IS_ZERO(exp.z(), 0.7580161194));

}


TEST_CASE("Matlab test 5", "[quaternion]") {
	Eigen::Quaterniond q1(0.6706829080, 0.0729356035, 0.5816342432, 0.4544958105);
	Eigen::Quaterniond q2(0.1419020229, 0.7786308280, 0.4898668719, 0.3655520447);
	Eigen::Vector3d v(0.9419189303, 0.6559138203, 0.4519457093);
	std::cout << "q1: " << q1 << std::endl;
	std::cout << "q2: " << q2 << std::endl;
	std::cout << "q1 (matlab): 0.6706829080, 0.0729356035, 0.5816342432, 0.4544958105"  << std::endl;
	std::cout << "q2 (matlab): 0.1419020229, 0.7786308280, 0.4898668719, 0.3655520447"  << std::endl;

	Eigen::Vector3d log = dmp::logarithmic_map(q1, q2);
	std::cout << "log (computed): " << log.transpose() << std::endl;
	std::cout << "log (matlab):  -0.5809705044, -0.6636209836, 0.2737614724" << std::endl;
	REQUIRE(IS_ZERO(log(0), -0.5809705044));
	REQUIRE(IS_ZERO(log(1), -0.6636209836));
	REQUIRE(IS_ZERO(log(2), 0.2737614724));

	Eigen::Quaterniond exp = dmp::exponential_map(v, q1);
	std::cout << "exponential (computed): " << exp << std::endl;
	std::cout << "exponential (matlab):  -0.2796276390, 0.5343637777, 0.2266998130, 0.7647685476" << std::endl;
	REQUIRE(IS_ZERO(exp.w(), -0.2796276390));
	REQUIRE(IS_ZERO(exp.x(), 0.5343637777));
	REQUIRE(IS_ZERO(exp.y(), 0.2266998130));
	REQUIRE(IS_ZERO(exp.z(), 0.7647685476));

}


