cmake_minimum_required(VERSION 3.0.2)
project(casadi_sample)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  casadi_eigen
  )

find_package(casadi REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED)

catkin_package(
  CATKIN_DEPENDS casadi_eigen
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
  )

add_executable(casadi_nlopt src/casadi_nlopt.cpp)
target_link_libraries(casadi_nlopt
  casadi
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${NLOPT_LIBRARIES}
  )

add_executable(casadi_sample src/casadi_sample.cpp)
target_link_libraries(casadi_sample
  casadi
  ${catkin_LIBRARIES}
  )


add_executable(casadi_sample2 src/casadi_sample2.cpp)
target_link_libraries(casadi_sample2
  casadi
  ${catkin_LIBRARIES}
  )

