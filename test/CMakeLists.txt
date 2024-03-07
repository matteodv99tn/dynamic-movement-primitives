# Download external dependencies
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

FetchContent_MakeAvailable(GnuPlot_iostream Catch2)


# Configure header with path definitions
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/common/defines.hpp.in
    ${CMAKE_CURRENT_SOURCE_DIR}/common/defines.hpp
)

# Create library with common function utilities
add_library(test_common common/test_functions.cpp)
target_include_directories(test_common PUBLIC ${CMAKE_CURRENT_LIST_DIR}/common)
target_link_libraries(test_common Eigen3::Eigen gnuplot_iostream dmp)


# Create executables for each cpp file
file(GLOB CPP_FILES *.cpp)
foreach(CPP_FILE ${CPP_FILES})
    get_filename_component(EXECUTABLE_NAME ${CPP_FILE} NAME_WE)
    add_executable(${EXECUTABLE_NAME} ${CPP_FILE})
    target_include_directories(${EXECUTABLE_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(${EXECUTABLE_NAME} dmp gnuplot_iostream test_common)
endforeach()


add_subdirectory(unit_test)
