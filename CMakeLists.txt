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
set(GnuPlotIostream_BuildTests OFF)
set(GnuPlotIostream_BuildExamples OFF)
FetchContent_Declare(
    GnuPlot_iostream
    GIT_REPOSITORY https://github.com/dstahlke/gnuplot-iostream.git
    )
FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.5.2
    )

FetchContent_MakeAvailable(range-v3 GnuPlot_iostream Catch2)

find_package(Eigen3 REQUIRED)

add_library(dmp STATIC
    src/basis_function/basis_function.cpp
    src/basis_function/periodic_gaussian_kernel.cpp
    src/dmp/quaternion_utils.cpp
    src/dmp/dmp_base.cpp
    src/dmp/periodic_dmp.cpp
    src/dmp/multidof_periodic_dmp.cpp
    src/dmp/quaternion_periodic_dmp.cpp
    src/dmp/utils.cpp
)
target_link_libraries(dmp Eigen3::Eigen range-v3)
target_include_directories(dmp
    PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include
    )

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/test)
