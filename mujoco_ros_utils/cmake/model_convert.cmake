macro(mujoco_model_convert _PROJECT_SOURCE_DIR _YAML_FILE)

  find_program(LSB_RELEASE_EXEC lsb_release)
  execute_process(COMMAND ${LSB_RELEASE_EXEC} -c
    OUTPUT_VARIABLE LSB_RELEASE_CODENAME
    )

  string(REPLACE "\t" ";" LSB_RELEASE_CODENAME_LIST ${LSB_RELEASE_CODENAME})
  list(GET LSB_RELEASE_CODENAME_LIST 1 CODENAME)

  find_program(UNAME_EXEC uname)
  execute_process(COMMAND ${UNAME_EXEC} -m
    OUTPUT_VARIABLE UNAME_OUTPUT
    )

  if(${CODENAME} MATCHES "xenial")
    MESSAGE(WARNING "Do not convert mujoco model in ${CODENAME}, because of old version of blender")

  elseif(${UNAME_OUTPUT} MATCHES "aarch64")
    MESSAGE(WARNING "Do not convert mujoco model in ${UNAME_OUTPUT}")

  else()

    add_custom_command(OUTPUT ${_PROJECT_SOURCE_DIR}/mujoco
      COMMAND rosrun mujoco_ros_control mujoco_model_generator.py ${_YAML_FILE}
      POST_BUILD
      WORKING_DIRECTORY ${_PROJECT_SOURCE_DIR})

    add_custom_target(generate_mujoco_model ALL DEPENDS ${_PROJECT_SOURCE_DIR}/mujoco)
  endif()
endmacro(mujoco_model_convert _PROJECT_SOURCE_DIR _YAML_FILE)
