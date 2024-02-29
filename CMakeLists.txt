cmake_minimum_required(VERSION 3.8)
project(dmps)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)

include(FetchContent)
FetchContent_Declare(
    range-v3
    GIT_REPOSITORY https://github.com/ericniebler/range-v3.git
    GIT_TAG 0.12.0
    )
FetchContent_MakeAvailable(range-v3)

find_package(Eigen3 REQUIRED)

include_directories(${CMAKE_CURRENT_LIST_DIR}/include)
link_libraries(Eigen3::Eigen range-v3)

add_library(dmp STATIC
    src/basis_function/basis_function.cpp
    src/basis_function/periodic_gaussian_kernel.cpp
    src/quaternion_utils.cpp
    src/dmp/dmp_base.cpp
    src/dmp/periodic_dmp.cpp
    src/dmp/multidof_periodic_dmp.cpp
    src/dmp/quaternion_periodic_dmp.cpp
    src/dmp/quaternion_periodic_dmp_v2.cpp
    src/dmp/utils.cpp
    )


add_executable(main src/main.cpp src/quaternion.cpp )
target_link_libraries(main dmp)
add_executable(test src/test.cpp)


add_executable(test_single_dmp test/single_dmp.cpp)
target_link_libraries(test_single_dmp dmp)
add_executable(test_single_dmp_incr test/single_dmp_incremental.cpp)
target_link_libraries(test_single_dmp_incr dmp)
add_executable(test_multiple_dmp test/multiple_dmp.cpp)
target_link_libraries(test_multiple_dmp dmp)

add_executable(test_multiple_dmp2 test/multiple_dmp2.cpp)
target_link_libraries(test_multiple_dmp2 dmp)

add_executable(quaternion_dmp test/quaternion_dmp.cpp)
target_link_libraries(quaternion_dmp dmp)
add_executable(quaternion_dmp2 test/quaternion_dmp2.cpp)
target_link_libraries(quaternion_dmp2 dmp)
