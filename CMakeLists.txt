cmake_minimum_required(VERSION 3.10)
project(sensor_fuse_cpp VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_EXAMPLES "Build demo applications" ON)
option(BUILD_TESTS "Build unit tests" ON)


add_library(sensor_fuse
    src/buffer.cpp
    src/kalman.cpp
    src/imu_service.cpp)

target_include_directories(sensor_fuse PUBLIC include)

if(BUILD_EXAMPLES)
  add_executable(demo examples/demo.cpp)
  target_link_libraries(demo sensor_fuse)
endif()

if(BUILD_TESTS)
  enable_testing()
  add_executable(test_buffer tests/test_buffer.cpp)
  target_link_libraries(test_buffer sensor_fuse)
  add_test(NAME test_buffer COMMAND test_buffer)

  add_executable(test_kalman tests/test_kalman.cpp)
  target_link_libraries(test_kalman sensor_fuse)
  add_test(NAME test_kalman COMMAND test_kalman)
endif()
