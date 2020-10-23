cmake_minimum_required(VERSION 2.8.3)
project(osqp-eigen)
include(ExternalProject)
find_package(osqp REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
execute_process(COMMAND
  git config --global http.sslVerify false)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
ExternalProject_Add(
  osqp-eigen
  GIT_REPOSITORY https://github.com/robotology/osqp-eigen
  GIT_TAG v0.6.2
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  INSTALL_COMMAND echo "install"
  DEPEND osqp
  )

install(CODE "execute_process(COMMAND make install WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/osqp-eigen-prefix/src/osqp-eigen-build)")
