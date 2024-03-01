file(GLOB CPP_FILES *.cpp)

add_executable(unit_tests ${CPP_FILES})
target_link_libraries(unit_tests Catch2 dmp)
