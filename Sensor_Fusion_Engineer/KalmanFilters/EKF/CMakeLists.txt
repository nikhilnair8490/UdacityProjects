cmake_minimum_required(VERSION 3.5)

# set project name
project(EKF)

# set C++ standard
set(CMAKE_CXX_STANDARD 11)

# include Eigen library
include_directories(/usr/include/eigen3)

# create executable
add_executable(EKF main.cpp kalman_filter.cpp tracking.cpp)
