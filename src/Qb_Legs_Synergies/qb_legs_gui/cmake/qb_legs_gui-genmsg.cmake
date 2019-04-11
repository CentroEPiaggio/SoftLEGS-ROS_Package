# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "qb_legs_gui: 5 messages, 0 services")

set(MSG_I_FLAGS "-Iqb_legs_gui:/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(qb_legs_gui_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/gain_bias_err.msg" NAME_WE)
add_custom_target(_qb_legs_gui_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "qb_legs_gui" "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/gain_bias_err.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/bias.msg" NAME_WE)
add_custom_target(_qb_legs_gui_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "qb_legs_gui" "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/bias.msg" ""
)

get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/data_corr_AT.msg" NAME_WE)
add_custom_target(_qb_legs_gui_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "qb_legs_gui" "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/data_corr_AT.msg" ""
)

get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/data_msg.msg" NAME_WE)
add_custom_target(_qb_legs_gui_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "qb_legs_gui" "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/data_msg.msg" ""
)

get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/GG_msg.msg" NAME_WE)
add_custom_target(_qb_legs_gui_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "qb_legs_gui" "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/GG_msg.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/gain_bias_err.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qb_legs_gui
)
_generate_msg_cpp(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/bias.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qb_legs_gui
)
_generate_msg_cpp(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/data_corr_AT.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qb_legs_gui
)
_generate_msg_cpp(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/GG_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qb_legs_gui
)
_generate_msg_cpp(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/data_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qb_legs_gui
)

### Generating Services

### Generating Module File
_generate_module_cpp(qb_legs_gui
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qb_legs_gui
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(qb_legs_gui_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(qb_legs_gui_generate_messages qb_legs_gui_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/gain_bias_err.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_cpp _qb_legs_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/bias.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_cpp _qb_legs_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/data_corr_AT.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_cpp _qb_legs_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/data_msg.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_cpp _qb_legs_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/GG_msg.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_cpp _qb_legs_gui_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qb_legs_gui_gencpp)
add_dependencies(qb_legs_gui_gencpp qb_legs_gui_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qb_legs_gui_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/gain_bias_err.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qb_legs_gui
)
_generate_msg_eus(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/bias.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qb_legs_gui
)
_generate_msg_eus(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/data_corr_AT.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qb_legs_gui
)
_generate_msg_eus(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/GG_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qb_legs_gui
)
_generate_msg_eus(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/data_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qb_legs_gui
)

### Generating Services

### Generating Module File
_generate_module_eus(qb_legs_gui
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qb_legs_gui
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(qb_legs_gui_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(qb_legs_gui_generate_messages qb_legs_gui_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/gain_bias_err.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_eus _qb_legs_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/bias.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_eus _qb_legs_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/data_corr_AT.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_eus _qb_legs_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/data_msg.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_eus _qb_legs_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/GG_msg.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_eus _qb_legs_gui_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qb_legs_gui_geneus)
add_dependencies(qb_legs_gui_geneus qb_legs_gui_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qb_legs_gui_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/gain_bias_err.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qb_legs_gui
)
_generate_msg_lisp(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/bias.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qb_legs_gui
)
_generate_msg_lisp(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/data_corr_AT.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qb_legs_gui
)
_generate_msg_lisp(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/GG_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qb_legs_gui
)
_generate_msg_lisp(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/data_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qb_legs_gui
)

### Generating Services

### Generating Module File
_generate_module_lisp(qb_legs_gui
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qb_legs_gui
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(qb_legs_gui_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(qb_legs_gui_generate_messages qb_legs_gui_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/gain_bias_err.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_lisp _qb_legs_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/bias.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_lisp _qb_legs_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/data_corr_AT.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_lisp _qb_legs_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/data_msg.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_lisp _qb_legs_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/GG_msg.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_lisp _qb_legs_gui_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qb_legs_gui_genlisp)
add_dependencies(qb_legs_gui_genlisp qb_legs_gui_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qb_legs_gui_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/gain_bias_err.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qb_legs_gui
)
_generate_msg_nodejs(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/bias.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qb_legs_gui
)
_generate_msg_nodejs(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/data_corr_AT.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qb_legs_gui
)
_generate_msg_nodejs(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/GG_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qb_legs_gui
)
_generate_msg_nodejs(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/data_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qb_legs_gui
)

### Generating Services

### Generating Module File
_generate_module_nodejs(qb_legs_gui
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qb_legs_gui
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(qb_legs_gui_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(qb_legs_gui_generate_messages qb_legs_gui_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/gain_bias_err.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_nodejs _qb_legs_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/bias.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_nodejs _qb_legs_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/data_corr_AT.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_nodejs _qb_legs_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/data_msg.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_nodejs _qb_legs_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/GG_msg.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_nodejs _qb_legs_gui_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qb_legs_gui_gennodejs)
add_dependencies(qb_legs_gui_gennodejs qb_legs_gui_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qb_legs_gui_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/gain_bias_err.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qb_legs_gui
)
_generate_msg_py(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/bias.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qb_legs_gui
)
_generate_msg_py(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/data_corr_AT.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qb_legs_gui
)
_generate_msg_py(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/GG_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qb_legs_gui
)
_generate_msg_py(qb_legs_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/data_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qb_legs_gui
)

### Generating Services

### Generating Module File
_generate_module_py(qb_legs_gui
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qb_legs_gui
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(qb_legs_gui_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(qb_legs_gui_generate_messages qb_legs_gui_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/gain_bias_err.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_py _qb_legs_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/bias.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_py _qb_legs_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/data_corr_AT.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_py _qb_legs_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/data_msg.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_py _qb_legs_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/qb_legs_gui/msg/GG_msg.msg" NAME_WE)
add_dependencies(qb_legs_gui_generate_messages_py _qb_legs_gui_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qb_legs_gui_genpy)
add_dependencies(qb_legs_gui_genpy qb_legs_gui_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qb_legs_gui_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qb_legs_gui)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qb_legs_gui
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(qb_legs_gui_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qb_legs_gui)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qb_legs_gui
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(qb_legs_gui_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qb_legs_gui)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qb_legs_gui
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(qb_legs_gui_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qb_legs_gui)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qb_legs_gui
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(qb_legs_gui_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qb_legs_gui)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qb_legs_gui\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qb_legs_gui
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(qb_legs_gui_generate_messages_py std_msgs_generate_messages_py)
endif()
