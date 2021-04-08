cmake_minimum_required(VERSION 3.16.3)
project(fusionFilter)


set(PROJECT_VERSION_MAJOR 0)
set(PROJECT_VERSION_MINOR 0)
set(PROJECT_VERSION_PATCH 1)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(SOURCE_FILES src/model.cpp src/ekf_test.cpp)
include_directories("include")
add_executable(ekf_test ${SOURCE_FILES})
