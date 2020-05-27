cmake_minimum_required(VERSION 2.8.3)
project(osqp_slsqp)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS)
find_package(Eigen3 REQUIRED COMPONENTS)
find_package(OsqpEigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES osqp_slsqp
  DEPENDS EIGEN3 OsqpEigen
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")

add_executable(sample_slsqp src/sample_slsqp.cpp)
target_link_libraries(sample_slsqp ${OsqpEigen_LIBRARIES})
