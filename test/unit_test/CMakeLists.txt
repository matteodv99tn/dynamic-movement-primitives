file(GLOB CPP_FILES *.cpp)

add_executable(unit_tests quaternions.cpp)
target_link_libraries(unit_tests Catch2 dmp)

add_executable(matlab_tests matlab.cpp)
target_link_libraries(matlab_tests Catch2 dmp)
