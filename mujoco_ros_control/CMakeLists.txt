cmake_minimum_required(VERSION 3.0.2)
project(mujoco_ros_control)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  mk
  pluginlib
  rosgraph_msgs
  roscpp
  sensor_msgs
  tf
  urdf
  )

set(MUJOCO mujoco)
add_custom_command(OUTPUT ${MUJOCO}_build
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  COMMAND $(MAKE) mujoco
  VERBATIM
  )
add_custom_target(${MUJOCO} DEPENDS ${MUJOCO}_build)

set(MUJOCO_INCLUDE_DIRS build/mujoco-2.3.7/include)
set(MUJOCO_LIBRARY ${PROJECT_SOURCE_DIR}/build/mujoco-2.3.7/lib/libmujoco.so)

catkin_package(
  INCLUDE_DIRS include ${MUJOCO_INCLUDE_DIRS}
  LIBRARIES ${MUJOCO_LIBRARY} mujoco_default_robot_hw_sim
  CATKIN_DEPENDS
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${MUJOCO_INCLUDE_DIRS}
  )

add_library(mujoco_default_robot_hw_sim
  src/mujoco_default_robot_hw_sim.cpp
  )
add_dependencies(mujoco_default_robot_hw_sim ${MUJOCO})
target_link_libraries(mujoco_default_robot_hw_sim ${catkin_LIBRARIES} ${MUJOCO_LIBRARY})

add_executable(mujoco_ros_control
  src/mujoco_ros_control.cpp
  src/mujoco_visualization_utils.cpp)
add_dependencies(mujoco_ros_control ${MUJOCO})
target_link_libraries(mujoco_ros_control
  ${catkin_LIBRARIES}
  glfw
  ${MUJOCO_LIBRARY}
  )

install(DIRECTORY include/${PROJECT_NAME}/ ${MUJOCO_INCLUDE_DIRS}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


install(TARGETS  mujoco_ros_control mujoco_default_robot_hw_sim
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(FILES mujoco_robot_hw_sim_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
