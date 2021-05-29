cmake_minimum_required(VERSION 3.13.4)
project(KalmanFilter)


set(PROJECT_VERSION_MAJOR 0)
set(PROJECT_VERSION_MINOR 0)
set(PROJECT_VERSION_PATCH 1)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories("include")
set(SOURCE_FILES src/model.cpp)
add_executable(kf_test tests/kf_test.cpp ${SOURCE_FILES})
add_executable(ekf_test tests/ekf_test.cpp ${SOURCE_FILES})
