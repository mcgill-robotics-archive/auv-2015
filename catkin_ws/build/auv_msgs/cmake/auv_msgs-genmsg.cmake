# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "auv_msgs: 7 messages, 0 services")

set(MSG_I_FLAGS "-Iauv_msgs:/home/vivi/auv/catkin_ws/src/auv_msgs/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(auv_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/Solenoid.msg" NAME_WE)
add_custom_target(_auv_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "auv_msgs" "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/Solenoid.msg" ""
)

get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/Motor.msg" NAME_WE)
add_custom_target(_auv_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "auv_msgs" "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/Motor.msg" ""
)

get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/MotorCommands.msg" NAME_WE)
add_custom_target(_auv_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "auv_msgs" "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/MotorCommands.msg" ""
)

get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/CVTarget.msg" NAME_WE)
add_custom_target(_auv_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "auv_msgs" "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/CVTarget.msg" ""
)

get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/SetVelocity.msg" NAME_WE)
add_custom_target(_auv_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "auv_msgs" "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/SetVelocity.msg" ""
)

get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/SonarTarget.msg" NAME_WE)
add_custom_target(_auv_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "auv_msgs" "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/SonarTarget.msg" ""
)

get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/SetPosition.msg" NAME_WE)
add_custom_target(_auv_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "auv_msgs" "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/SetPosition.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(auv_msgs
  "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/Solenoid.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/auv_msgs
)
_generate_msg_cpp(auv_msgs
  "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/Motor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/auv_msgs
)
_generate_msg_cpp(auv_msgs
  "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/MotorCommands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/auv_msgs
)
_generate_msg_cpp(auv_msgs
  "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/CVTarget.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/auv_msgs
)
_generate_msg_cpp(auv_msgs
  "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/SetVelocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/auv_msgs
)
_generate_msg_cpp(auv_msgs
  "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/SonarTarget.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/auv_msgs
)
_generate_msg_cpp(auv_msgs
  "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/SetPosition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/auv_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(auv_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/auv_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(auv_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(auv_msgs_generate_messages auv_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/Solenoid.msg" NAME_WE)
add_dependencies(auv_msgs_generate_messages_cpp _auv_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/Motor.msg" NAME_WE)
add_dependencies(auv_msgs_generate_messages_cpp _auv_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/MotorCommands.msg" NAME_WE)
add_dependencies(auv_msgs_generate_messages_cpp _auv_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/CVTarget.msg" NAME_WE)
add_dependencies(auv_msgs_generate_messages_cpp _auv_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/SetVelocity.msg" NAME_WE)
add_dependencies(auv_msgs_generate_messages_cpp _auv_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/SonarTarget.msg" NAME_WE)
add_dependencies(auv_msgs_generate_messages_cpp _auv_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/SetPosition.msg" NAME_WE)
add_dependencies(auv_msgs_generate_messages_cpp _auv_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(auv_msgs_gencpp)
add_dependencies(auv_msgs_gencpp auv_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS auv_msgs_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(auv_msgs
  "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/Solenoid.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/auv_msgs
)
_generate_msg_lisp(auv_msgs
  "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/Motor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/auv_msgs
)
_generate_msg_lisp(auv_msgs
  "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/MotorCommands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/auv_msgs
)
_generate_msg_lisp(auv_msgs
  "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/CVTarget.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/auv_msgs
)
_generate_msg_lisp(auv_msgs
  "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/SetVelocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/auv_msgs
)
_generate_msg_lisp(auv_msgs
  "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/SonarTarget.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/auv_msgs
)
_generate_msg_lisp(auv_msgs
  "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/SetPosition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/auv_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(auv_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/auv_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(auv_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(auv_msgs_generate_messages auv_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/Solenoid.msg" NAME_WE)
add_dependencies(auv_msgs_generate_messages_lisp _auv_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/Motor.msg" NAME_WE)
add_dependencies(auv_msgs_generate_messages_lisp _auv_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/MotorCommands.msg" NAME_WE)
add_dependencies(auv_msgs_generate_messages_lisp _auv_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/CVTarget.msg" NAME_WE)
add_dependencies(auv_msgs_generate_messages_lisp _auv_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/SetVelocity.msg" NAME_WE)
add_dependencies(auv_msgs_generate_messages_lisp _auv_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/SonarTarget.msg" NAME_WE)
add_dependencies(auv_msgs_generate_messages_lisp _auv_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/SetPosition.msg" NAME_WE)
add_dependencies(auv_msgs_generate_messages_lisp _auv_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(auv_msgs_genlisp)
add_dependencies(auv_msgs_genlisp auv_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS auv_msgs_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(auv_msgs
  "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/Solenoid.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/auv_msgs
)
_generate_msg_py(auv_msgs
  "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/Motor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/auv_msgs
)
_generate_msg_py(auv_msgs
  "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/MotorCommands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/auv_msgs
)
_generate_msg_py(auv_msgs
  "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/CVTarget.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/auv_msgs
)
_generate_msg_py(auv_msgs
  "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/SetVelocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/auv_msgs
)
_generate_msg_py(auv_msgs
  "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/SonarTarget.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/auv_msgs
)
_generate_msg_py(auv_msgs
  "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/SetPosition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/auv_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(auv_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/auv_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(auv_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(auv_msgs_generate_messages auv_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/Solenoid.msg" NAME_WE)
add_dependencies(auv_msgs_generate_messages_py _auv_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/Motor.msg" NAME_WE)
add_dependencies(auv_msgs_generate_messages_py _auv_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/MotorCommands.msg" NAME_WE)
add_dependencies(auv_msgs_generate_messages_py _auv_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/CVTarget.msg" NAME_WE)
add_dependencies(auv_msgs_generate_messages_py _auv_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/SetVelocity.msg" NAME_WE)
add_dependencies(auv_msgs_generate_messages_py _auv_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/SonarTarget.msg" NAME_WE)
add_dependencies(auv_msgs_generate_messages_py _auv_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivi/auv/catkin_ws/src/auv_msgs/msg/SetPosition.msg" NAME_WE)
add_dependencies(auv_msgs_generate_messages_py _auv_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(auv_msgs_genpy)
add_dependencies(auv_msgs_genpy auv_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS auv_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/auv_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/auv_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/auv_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/auv_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/auv_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/auv_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/auv_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
