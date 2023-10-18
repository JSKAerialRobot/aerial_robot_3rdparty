macro(mujoco_model_convert _PROJECT_SOURCE_DIR _URDF_DIR _YAML_FILE)
  add_custom_command(OUTPUT ${_PROJECT_SOURCE_DIR}/mujoco
    COMMAND rosrun mujoco_ros_control mujoco_model_generator.py ${_YAML_FILE}
    POST_BUILD
    WORKING_DIRECTORY ${_PROJECT_SOURCE_DIR}
    DEPENDS ${_URDF_DIR}  ${_YAML_FILE})

  add_custom_target(generate_mujoco_model ALL DEPENDS ${_PROJECT_SOURCE_DIR}/mujoco)
endmacro(mujoco_model_convert _PROJECT_SOURCE_DIR)
