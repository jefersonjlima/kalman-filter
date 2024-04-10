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
set(KF_SOURCE_FILES src/model.cpp tests/kf_test.cpp)
add_executable(kf_test ${KF_SOURCE_FILES})
set(EKF_SOURCE_FILES src/model.cpp tests/ekf_test.cpp)
add_executable(ekf_test tests/ekf_test.cpp ${EKF_SOURCE_FILES})
