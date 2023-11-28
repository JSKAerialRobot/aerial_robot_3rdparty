cmake_minimum_required(VERSION 2.8.3)
project(casadi_sample)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS)
find_package(casadi REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${casadi_INCLUDE_DIRS}
)

add_executable(casadi_sample src/casadi_sample.cpp)
target_link_libraries(casadi_sample casadi)
