cmake_minimum_required(VERSION 3.8)
project(dmplib)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


set(CMAKE_CXX_STANDARD 20)
set(DMPLIB_BUILD_TESTS OFF CACHE BOOL "Build tests")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)

include(FetchContent)
# FetchContent_Declare(
#     range-v3
#     GIT_REPOSITORY https://github.com/ericniebler/range-v3.git
#     GIT_TAG 0.12.0
#     )
# FetchContent_MakeAvailable(range-v3)

find_package(Eigen3 REQUIRED)

add_library(dmp INTERFACE
    )
target_link_libraries(dmp INTERFACE Eigen3::Eigen) # range-v3)
target_include_directories(dmp
INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include
    )

if(DMPLIB_BUILD_TESTS)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/test)

add_executable(example example.cpp)
target_link_libraries(example Eigen3::Eigen  dmp)

add_executable(example2 example.cpp)
target_link_libraries(example2 Eigen3::Eigen  dmp)
endif()
