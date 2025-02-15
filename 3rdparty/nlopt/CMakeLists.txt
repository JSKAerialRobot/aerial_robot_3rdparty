cmake_minimum_required(VERSION 2.8.3)
project(nlopt)
include(ExternalProject)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_program(LSB_RELEASE_EXEC lsb_release)
execute_process(COMMAND ${LSB_RELEASE_EXEC} -c
    OUTPUT_VARIABLE LSB_RELEASE_CODENAME
)

string(REPLACE "\t" ";" LSB_RELEASE_CODENAME_LIST ${LSB_RELEASE_CODENAME})
list(GET LSB_RELEASE_CODENAME_LIST 1 CODENAME)

if(${CODENAME} MATCHES "bionic")
  set(GIT_TAG v2.6.0)
else()
  set(GIT_TAG v2.9.0)
endif()

execute_process(COMMAND
  git config --global http.sslVerify false)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
ExternalProject_Add(
  nlopt
  GIT_REPOSITORY https://github.com/stevengj/nlopt.git
  GIT_TAG ${GIT_TAG}
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  INSTALL_COMMAND echo "install"
  )

install(CODE "execute_process(COMMAND make install WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/nlopt-prefix/src/nlopt-build)")
