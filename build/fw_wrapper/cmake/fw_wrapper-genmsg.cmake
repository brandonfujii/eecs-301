# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "fw_wrapper: 1 messages, 2 services")

set(MSG_I_FLAGS "-Ifw_wrapper:/home/rosuser/ros_workspace/src/fw_wrapper/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(fw_wrapper_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rosuser/ros_workspace/src/fw_wrapper/srv/getcmd.srv" NAME_WE)
add_custom_target(_fw_wrapper_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fw_wrapper" "/home/rosuser/ros_workspace/src/fw_wrapper/srv/getcmd.srv" ""
)

get_filename_component(_filename "/home/rosuser/ros_workspace/src/fw_wrapper/srv/allcmd.srv" NAME_WE)
add_custom_target(_fw_wrapper_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fw_wrapper" "/home/rosuser/ros_workspace/src/fw_wrapper/srv/allcmd.srv" ""
)

get_filename_component(_filename "/home/rosuser/ros_workspace/src/fw_wrapper/msg/command.msg" NAME_WE)
add_custom_target(_fw_wrapper_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fw_wrapper" "/home/rosuser/ros_workspace/src/fw_wrapper/msg/command.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(fw_wrapper
  "/home/rosuser/ros_workspace/src/fw_wrapper/msg/command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fw_wrapper
)

### Generating Services
_generate_srv_cpp(fw_wrapper
  "/home/rosuser/ros_workspace/src/fw_wrapper/srv/getcmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fw_wrapper
)
_generate_srv_cpp(fw_wrapper
  "/home/rosuser/ros_workspace/src/fw_wrapper/srv/allcmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fw_wrapper
)

### Generating Module File
_generate_module_cpp(fw_wrapper
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fw_wrapper
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(fw_wrapper_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(fw_wrapper_generate_messages fw_wrapper_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rosuser/ros_workspace/src/fw_wrapper/srv/getcmd.srv" NAME_WE)
add_dependencies(fw_wrapper_generate_messages_cpp _fw_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rosuser/ros_workspace/src/fw_wrapper/srv/allcmd.srv" NAME_WE)
add_dependencies(fw_wrapper_generate_messages_cpp _fw_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rosuser/ros_workspace/src/fw_wrapper/msg/command.msg" NAME_WE)
add_dependencies(fw_wrapper_generate_messages_cpp _fw_wrapper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fw_wrapper_gencpp)
add_dependencies(fw_wrapper_gencpp fw_wrapper_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fw_wrapper_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(fw_wrapper
  "/home/rosuser/ros_workspace/src/fw_wrapper/msg/command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fw_wrapper
)

### Generating Services
_generate_srv_lisp(fw_wrapper
  "/home/rosuser/ros_workspace/src/fw_wrapper/srv/getcmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fw_wrapper
)
_generate_srv_lisp(fw_wrapper
  "/home/rosuser/ros_workspace/src/fw_wrapper/srv/allcmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fw_wrapper
)

### Generating Module File
_generate_module_lisp(fw_wrapper
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fw_wrapper
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(fw_wrapper_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(fw_wrapper_generate_messages fw_wrapper_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rosuser/ros_workspace/src/fw_wrapper/srv/getcmd.srv" NAME_WE)
add_dependencies(fw_wrapper_generate_messages_lisp _fw_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rosuser/ros_workspace/src/fw_wrapper/srv/allcmd.srv" NAME_WE)
add_dependencies(fw_wrapper_generate_messages_lisp _fw_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rosuser/ros_workspace/src/fw_wrapper/msg/command.msg" NAME_WE)
add_dependencies(fw_wrapper_generate_messages_lisp _fw_wrapper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fw_wrapper_genlisp)
add_dependencies(fw_wrapper_genlisp fw_wrapper_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fw_wrapper_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(fw_wrapper
  "/home/rosuser/ros_workspace/src/fw_wrapper/msg/command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fw_wrapper
)

### Generating Services
_generate_srv_py(fw_wrapper
  "/home/rosuser/ros_workspace/src/fw_wrapper/srv/getcmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fw_wrapper
)
_generate_srv_py(fw_wrapper
  "/home/rosuser/ros_workspace/src/fw_wrapper/srv/allcmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fw_wrapper
)

### Generating Module File
_generate_module_py(fw_wrapper
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fw_wrapper
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(fw_wrapper_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(fw_wrapper_generate_messages fw_wrapper_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rosuser/ros_workspace/src/fw_wrapper/srv/getcmd.srv" NAME_WE)
add_dependencies(fw_wrapper_generate_messages_py _fw_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rosuser/ros_workspace/src/fw_wrapper/srv/allcmd.srv" NAME_WE)
add_dependencies(fw_wrapper_generate_messages_py _fw_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rosuser/ros_workspace/src/fw_wrapper/msg/command.msg" NAME_WE)
add_dependencies(fw_wrapper_generate_messages_py _fw_wrapper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fw_wrapper_genpy)
add_dependencies(fw_wrapper_genpy fw_wrapper_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fw_wrapper_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fw_wrapper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fw_wrapper
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(fw_wrapper_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fw_wrapper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fw_wrapper
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(fw_wrapper_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fw_wrapper)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fw_wrapper\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fw_wrapper
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(fw_wrapper_generate_messages_py std_msgs_generate_messages_py)
