cmake_minimum_required(VERSION 3.8)
# set(CXX_CLANG_TIDY ON)
project(dmplib)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

set(DMPLIB_BUILD_TESTS ON CACHE BOOL "Build tests")
set(DMPLIB_RUN_CLANG_TIDY ON CACHE BOOL "Automatically run clang-tidy on each source code")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

find_package(Eigen3 REQUIRED)

include(FetchContent)
FetchContent_Declare(
    range-v3
    URL https://github.com/ericniebler/range-v3/archive/refs/tags/0.12.0.zip
)
FetchContent_Declare(
    fmt
    URL https://github.com/fmtlib/fmt/archive/refs/tags/10.2.1.zip
)
FetchContent_MakeAvailable(range-v3 fmt)

# Matplotlib interface
add_library(matplotlib_cpp INTERFACE)
find_package(Python3 COMPONENTS Interpreter Development)
if(${Python3_FOUND})
    target_link_libraries(
        matplotlib_cpp 
        INTERFACE
        Python3::Python
        Python3::Module
    )
    message(STATUS "Python3 found, matplotlib_cpp will be available")
    target_include_directories(matplotlib_cpp INTERFACE ${CMAKE_CURRENT_BINARY_DIR}/external_includes)
    target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY MATPLOTLIBAVAILABLE)

    set(MATPLOTLIB_H_DOWNLOAD_PATH ${CMAKE_CURRENT_BINARY_DIR}/external_includes/matplotlibcpp.h)
    if (NOT EXISTS "${MATPLOTLIB_H_DOWNLOAD_PATH}")
        message(STATUS "Downloading matplotlib_cpp header")
        file(DOWNLOAD "https://raw.githubusercontent.com/lava/matplotlib-cpp/master/matplotlibcpp.h" "${MATPLOTLIB_H_DOWNLOAD_PATH}")
    endif()
else()
    message(WARNING "Python3 not found, so won't be able to use matplotlib_cpp")
endif()

if(DMPLIB_RUN_CLANG_TIDY)
    find_program(CLANGTIDY clang-tidy)
    set(CMAKE_CXX_CLANG_TIDY
        clang-tidy --config-file=${CMAKE_CURRENT_LIST_DIR}/.clang-tidy
        )
    if(CLANGTIDY)
        message(STATUS "Running clang-tidy on each project source")
    else()
        message(WARNING "Unable to find clang-tidy; can't perform static analysis")
    endif()
else()
    message(STATUS "Skipping clang-tidy")
endif()


add_library(
    ${PROJECT_NAME}
    STATIC
    src/manifolds/s3_manifold.cpp
    src/manifolds/se3_manifold.cpp
    src/coordinate_systems/periodic_coordinate_system.cpp
    src/coordinate_systems/exponential_decay_cs.cpp
    src/basis_functions/gaussian_bf.cpp
    src/basis_functions/radial_gaussian_bf.cpp
    src/data_handler/conversions.cpp
)
target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC 
    Eigen3::Eigen 
    range-v3 
    fmt::fmt
    matplotlib_cpp
)
target_include_directories(
    ${PROJECT_NAME}
    PUBLIC 
    ${CMAKE_CURRENT_LIST_DIR}/include
)
target_compile_options(
    ${PROJECT_NAME}
    PRIVATE 
    -Wall -Wextra
)

if(DMPLIB_BUILD_TESTS)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/tests)
endif()

set(DMPLIB_AUTOMATIC_STATIC_CHECK OFF)

find_program(CPPCHECK cppcheck)
if(CPPCHECK)
    # Retrieve targets
    get_target_property(dmp_sources ${PROJECT_NAME} SOURCES)
    message(VERBOSE "Registering following sources for cppcheck:")
    foreach (s ${sources})
        message(VERBOSE " > ${s}")
    endforeach()

    add_custom_target(cppcheck
        COMMAND ${CPPCHECK}
                --enable=all
                --inconclusive
                --quiet
                --suppress=missingIncludeSystem
                --project=${CMAKE_BINARY_DIR}/compile_commands.json
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Running cppcheck"
    )

    if(DMPLIB_AUTOMATIC_CPPCHECK)
        add_dependencies(${PROJECT_NAME} cppcheck)
    endif()
else()
    message(STATUS "cppcheck not found, skipping static check")
endif()
