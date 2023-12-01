cmake_minimum_required(VERSION 3.0.2)
project(casadi)
include(ExternalProject)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
execute_process(COMMAND
  git config --global http.sslVerify false)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
ExternalProject_Add(
  casadi
  GIT_REPOSITORY https://github.com/casadi/casadi
  GIT_TAG 3.6.4
  CMAKE_ARGS -DWITH_IPOPT=ON -DWITH_LAPACK=ON -DWITH_OSQP=ON -DWITH_QPOASES=ON -DWITH_PYTHON=OFF -DWITH_PYTHON3=OFF -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  INSTALL_COMMAND echo "install"
  )

install(CODE "execute_process(COMMAND make install WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/casadi-prefix/src/casadi-build)")
