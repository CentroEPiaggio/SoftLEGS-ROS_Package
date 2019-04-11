# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "phidgets_interface_kit: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iphidgets_interface_kit:/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(phidgets_interface_kit_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg/AnalogArray.msg" NAME_WE)
add_custom_target(_phidgets_interface_kit_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "phidgets_interface_kit" "/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg/AnalogArray.msg" ""
)

get_filename_component(_filename "/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg/DigitalArray.msg" NAME_WE)
add_custom_target(_phidgets_interface_kit_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "phidgets_interface_kit" "/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg/DigitalArray.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(phidgets_interface_kit
  "/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg/AnalogArray.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phidgets_interface_kit
)
_generate_msg_cpp(phidgets_interface_kit
  "/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg/DigitalArray.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phidgets_interface_kit
)

### Generating Services

### Generating Module File
_generate_module_cpp(phidgets_interface_kit
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phidgets_interface_kit
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(phidgets_interface_kit_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(phidgets_interface_kit_generate_messages phidgets_interface_kit_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg/AnalogArray.msg" NAME_WE)
add_dependencies(phidgets_interface_kit_generate_messages_cpp _phidgets_interface_kit_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg/DigitalArray.msg" NAME_WE)
add_dependencies(phidgets_interface_kit_generate_messages_cpp _phidgets_interface_kit_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(phidgets_interface_kit_gencpp)
add_dependencies(phidgets_interface_kit_gencpp phidgets_interface_kit_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS phidgets_interface_kit_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(phidgets_interface_kit
  "/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg/AnalogArray.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/phidgets_interface_kit
)
_generate_msg_eus(phidgets_interface_kit
  "/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg/DigitalArray.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/phidgets_interface_kit
)

### Generating Services

### Generating Module File
_generate_module_eus(phidgets_interface_kit
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/phidgets_interface_kit
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(phidgets_interface_kit_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(phidgets_interface_kit_generate_messages phidgets_interface_kit_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg/AnalogArray.msg" NAME_WE)
add_dependencies(phidgets_interface_kit_generate_messages_eus _phidgets_interface_kit_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg/DigitalArray.msg" NAME_WE)
add_dependencies(phidgets_interface_kit_generate_messages_eus _phidgets_interface_kit_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(phidgets_interface_kit_geneus)
add_dependencies(phidgets_interface_kit_geneus phidgets_interface_kit_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS phidgets_interface_kit_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(phidgets_interface_kit
  "/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg/AnalogArray.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phidgets_interface_kit
)
_generate_msg_lisp(phidgets_interface_kit
  "/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg/DigitalArray.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phidgets_interface_kit
)

### Generating Services

### Generating Module File
_generate_module_lisp(phidgets_interface_kit
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phidgets_interface_kit
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(phidgets_interface_kit_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(phidgets_interface_kit_generate_messages phidgets_interface_kit_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg/AnalogArray.msg" NAME_WE)
add_dependencies(phidgets_interface_kit_generate_messages_lisp _phidgets_interface_kit_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg/DigitalArray.msg" NAME_WE)
add_dependencies(phidgets_interface_kit_generate_messages_lisp _phidgets_interface_kit_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(phidgets_interface_kit_genlisp)
add_dependencies(phidgets_interface_kit_genlisp phidgets_interface_kit_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS phidgets_interface_kit_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(phidgets_interface_kit
  "/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg/AnalogArray.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/phidgets_interface_kit
)
_generate_msg_nodejs(phidgets_interface_kit
  "/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg/DigitalArray.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/phidgets_interface_kit
)

### Generating Services

### Generating Module File
_generate_module_nodejs(phidgets_interface_kit
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/phidgets_interface_kit
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(phidgets_interface_kit_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(phidgets_interface_kit_generate_messages phidgets_interface_kit_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg/AnalogArray.msg" NAME_WE)
add_dependencies(phidgets_interface_kit_generate_messages_nodejs _phidgets_interface_kit_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg/DigitalArray.msg" NAME_WE)
add_dependencies(phidgets_interface_kit_generate_messages_nodejs _phidgets_interface_kit_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(phidgets_interface_kit_gennodejs)
add_dependencies(phidgets_interface_kit_gennodejs phidgets_interface_kit_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS phidgets_interface_kit_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(phidgets_interface_kit
  "/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg/AnalogArray.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phidgets_interface_kit
)
_generate_msg_py(phidgets_interface_kit
  "/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg/DigitalArray.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phidgets_interface_kit
)

### Generating Services

### Generating Module File
_generate_module_py(phidgets_interface_kit
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phidgets_interface_kit
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(phidgets_interface_kit_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(phidgets_interface_kit_generate_messages phidgets_interface_kit_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg/AnalogArray.msg" NAME_WE)
add_dependencies(phidgets_interface_kit_generate_messages_py _phidgets_interface_kit_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/phidgets_interface_kit/msg/DigitalArray.msg" NAME_WE)
add_dependencies(phidgets_interface_kit_generate_messages_py _phidgets_interface_kit_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(phidgets_interface_kit_genpy)
add_dependencies(phidgets_interface_kit_genpy phidgets_interface_kit_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS phidgets_interface_kit_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phidgets_interface_kit)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phidgets_interface_kit
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(phidgets_interface_kit_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(phidgets_interface_kit_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/phidgets_interface_kit)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/phidgets_interface_kit
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(phidgets_interface_kit_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(phidgets_interface_kit_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phidgets_interface_kit)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phidgets_interface_kit
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(phidgets_interface_kit_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(phidgets_interface_kit_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/phidgets_interface_kit)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/phidgets_interface_kit
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(phidgets_interface_kit_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(phidgets_interface_kit_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phidgets_interface_kit)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phidgets_interface_kit\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phidgets_interface_kit
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(phidgets_interface_kit_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(phidgets_interface_kit_generate_messages_py sensor_msgs_generate_messages_py)
endif()
