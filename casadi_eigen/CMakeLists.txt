cmake_minimum_required(VERSION 3.0.2)
project(casadi_eigen)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS)

find_package(casadi REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES casadi_eigen
  )

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  )

add_library(casadi_eigen src/CasadiEigen.cpp)
target_link_libraries(casadi_eigen
  casadi
  ${EIGEN3_LIBRARIES}
  )
