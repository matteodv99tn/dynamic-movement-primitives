file(GLOB CPP_FILES *.cpp)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/common/defines.hpp.in
    ${CMAKE_CURRENT_SOURCE_DIR}/common/defines.hpp
)

foreach(CPP_FILE ${CPP_FILES})
    get_filename_component(EXECUTABLE_NAME ${CPP_FILE} NAME_WE)
    add_executable(${EXECUTABLE_NAME} ${CPP_FILE})
    target_include_directories(${EXECUTABLE_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(${EXECUTABLE_NAME} dmp gnuplot_iostream)
endforeach()


add_subdirectory(unit_test)
