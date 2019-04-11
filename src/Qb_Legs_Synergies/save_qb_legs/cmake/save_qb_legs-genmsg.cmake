# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "save_qb_legs: 1 messages, 0 services")

set(MSG_I_FLAGS "-Isave_qb_legs:/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/save_qb_legs/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(save_qb_legs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/save_qb_legs/msg/data_msg.msg" NAME_WE)
add_custom_target(_save_qb_legs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "save_qb_legs" "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/save_qb_legs/msg/data_msg.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(save_qb_legs
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/save_qb_legs/msg/data_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/save_qb_legs
)

### Generating Services

### Generating Module File
_generate_module_cpp(save_qb_legs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/save_qb_legs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(save_qb_legs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(save_qb_legs_generate_messages save_qb_legs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/save_qb_legs/msg/data_msg.msg" NAME_WE)
add_dependencies(save_qb_legs_generate_messages_cpp _save_qb_legs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(save_qb_legs_gencpp)
add_dependencies(save_qb_legs_gencpp save_qb_legs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS save_qb_legs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(save_qb_legs
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/save_qb_legs/msg/data_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/save_qb_legs
)

### Generating Services

### Generating Module File
_generate_module_eus(save_qb_legs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/save_qb_legs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(save_qb_legs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(save_qb_legs_generate_messages save_qb_legs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/save_qb_legs/msg/data_msg.msg" NAME_WE)
add_dependencies(save_qb_legs_generate_messages_eus _save_qb_legs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(save_qb_legs_geneus)
add_dependencies(save_qb_legs_geneus save_qb_legs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS save_qb_legs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(save_qb_legs
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/save_qb_legs/msg/data_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/save_qb_legs
)

### Generating Services

### Generating Module File
_generate_module_lisp(save_qb_legs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/save_qb_legs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(save_qb_legs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(save_qb_legs_generate_messages save_qb_legs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/save_qb_legs/msg/data_msg.msg" NAME_WE)
add_dependencies(save_qb_legs_generate_messages_lisp _save_qb_legs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(save_qb_legs_genlisp)
add_dependencies(save_qb_legs_genlisp save_qb_legs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS save_qb_legs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(save_qb_legs
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/save_qb_legs/msg/data_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/save_qb_legs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(save_qb_legs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/save_qb_legs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(save_qb_legs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(save_qb_legs_generate_messages save_qb_legs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/save_qb_legs/msg/data_msg.msg" NAME_WE)
add_dependencies(save_qb_legs_generate_messages_nodejs _save_qb_legs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(save_qb_legs_gennodejs)
add_dependencies(save_qb_legs_gennodejs save_qb_legs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS save_qb_legs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(save_qb_legs
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/save_qb_legs/msg/data_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/save_qb_legs
)

### Generating Services

### Generating Module File
_generate_module_py(save_qb_legs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/save_qb_legs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(save_qb_legs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(save_qb_legs_generate_messages save_qb_legs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/save_qb_legs/msg/data_msg.msg" NAME_WE)
add_dependencies(save_qb_legs_generate_messages_py _save_qb_legs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(save_qb_legs_genpy)
add_dependencies(save_qb_legs_genpy save_qb_legs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS save_qb_legs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/save_qb_legs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/save_qb_legs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(save_qb_legs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/save_qb_legs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/save_qb_legs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(save_qb_legs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/save_qb_legs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/save_qb_legs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(save_qb_legs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/save_qb_legs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/save_qb_legs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(save_qb_legs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/save_qb_legs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/save_qb_legs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/save_qb_legs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(save_qb_legs_generate_messages_py std_msgs_generate_messages_py)
endif()
