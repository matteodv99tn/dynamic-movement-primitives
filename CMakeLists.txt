cmake_minimum_required(VERSION 3.8)
project(dmplib)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


set(CMAKE_CXX_STANDARD 17)
set(DMPLIB_BUILD_TESTS ON CACHE BOOL "Build tests")
# set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")

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

add_library(dmp STATIC
    src/manifolds/s3_manifold.cpp
    src/manifolds/se3_manifold.cpp
    )
target_link_libraries(dmp PUBLIC Eigen3::Eigen range-v3)
target_include_directories(dmp
    PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include
    )
target_compile_options(dmp PRIVATE -Wall -Wextra)


if(DMPLIB_BUILD_TESTS)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/tests)
endif()