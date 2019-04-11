# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "optimal_walk_gui: 6 messages, 0 services")

set(MSG_I_FLAGS "-Ioptimal_walk_gui:/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(optimal_walk_gui_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/gain_bias_err.msg" NAME_WE)
add_custom_target(_optimal_walk_gui_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "optimal_walk_gui" "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/gain_bias_err.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/GG_msg.msg" NAME_WE)
add_custom_target(_optimal_walk_gui_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "optimal_walk_gui" "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/GG_msg.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/dataset_name.msg" NAME_WE)
add_custom_target(_optimal_walk_gui_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "optimal_walk_gui" "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/dataset_name.msg" ""
)

get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/data_corr_AT.msg" NAME_WE)
add_custom_target(_optimal_walk_gui_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "optimal_walk_gui" "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/data_corr_AT.msg" ""
)

get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/bias.msg" NAME_WE)
add_custom_target(_optimal_walk_gui_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "optimal_walk_gui" "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/bias.msg" ""
)

get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/data_msg.msg" NAME_WE)
add_custom_target(_optimal_walk_gui_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "optimal_walk_gui" "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/data_msg.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/gain_bias_err.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_cpp(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/GG_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_cpp(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/dataset_name.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_cpp(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/data_corr_AT.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_cpp(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/bias.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_cpp(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/data_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/optimal_walk_gui
)

### Generating Services

### Generating Module File
_generate_module_cpp(optimal_walk_gui
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/optimal_walk_gui
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(optimal_walk_gui_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(optimal_walk_gui_generate_messages optimal_walk_gui_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/gain_bias_err.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_cpp _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/GG_msg.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_cpp _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/dataset_name.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_cpp _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/data_corr_AT.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_cpp _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/bias.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_cpp _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/data_msg.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_cpp _optimal_walk_gui_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(optimal_walk_gui_gencpp)
add_dependencies(optimal_walk_gui_gencpp optimal_walk_gui_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS optimal_walk_gui_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/gain_bias_err.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_eus(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/GG_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_eus(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/dataset_name.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_eus(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/data_corr_AT.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_eus(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/bias.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_eus(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/data_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/optimal_walk_gui
)

### Generating Services

### Generating Module File
_generate_module_eus(optimal_walk_gui
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/optimal_walk_gui
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(optimal_walk_gui_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(optimal_walk_gui_generate_messages optimal_walk_gui_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/gain_bias_err.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_eus _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/GG_msg.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_eus _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/dataset_name.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_eus _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/data_corr_AT.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_eus _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/bias.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_eus _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/data_msg.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_eus _optimal_walk_gui_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(optimal_walk_gui_geneus)
add_dependencies(optimal_walk_gui_geneus optimal_walk_gui_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS optimal_walk_gui_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/gain_bias_err.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_lisp(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/GG_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_lisp(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/dataset_name.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_lisp(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/data_corr_AT.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_lisp(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/bias.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_lisp(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/data_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/optimal_walk_gui
)

### Generating Services

### Generating Module File
_generate_module_lisp(optimal_walk_gui
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/optimal_walk_gui
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(optimal_walk_gui_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(optimal_walk_gui_generate_messages optimal_walk_gui_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/gain_bias_err.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_lisp _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/GG_msg.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_lisp _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/dataset_name.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_lisp _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/data_corr_AT.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_lisp _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/bias.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_lisp _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/data_msg.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_lisp _optimal_walk_gui_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(optimal_walk_gui_genlisp)
add_dependencies(optimal_walk_gui_genlisp optimal_walk_gui_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS optimal_walk_gui_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/gain_bias_err.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_nodejs(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/GG_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_nodejs(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/dataset_name.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_nodejs(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/data_corr_AT.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_nodejs(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/bias.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_nodejs(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/data_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/optimal_walk_gui
)

### Generating Services

### Generating Module File
_generate_module_nodejs(optimal_walk_gui
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/optimal_walk_gui
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(optimal_walk_gui_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(optimal_walk_gui_generate_messages optimal_walk_gui_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/gain_bias_err.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_nodejs _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/GG_msg.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_nodejs _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/dataset_name.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_nodejs _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/data_corr_AT.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_nodejs _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/bias.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_nodejs _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/data_msg.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_nodejs _optimal_walk_gui_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(optimal_walk_gui_gennodejs)
add_dependencies(optimal_walk_gui_gennodejs optimal_walk_gui_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS optimal_walk_gui_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/gain_bias_err.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_py(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/GG_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_py(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/dataset_name.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_py(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/data_corr_AT.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_py(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/bias.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/optimal_walk_gui
)
_generate_msg_py(optimal_walk_gui
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/data_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/optimal_walk_gui
)

### Generating Services

### Generating Module File
_generate_module_py(optimal_walk_gui
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/optimal_walk_gui
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(optimal_walk_gui_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(optimal_walk_gui_generate_messages optimal_walk_gui_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/gain_bias_err.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_py _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/GG_msg.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_py _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/dataset_name.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_py _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/data_corr_AT.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_py _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/bias.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_py _optimal_walk_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk_gui/msg/data_msg.msg" NAME_WE)
add_dependencies(optimal_walk_gui_generate_messages_py _optimal_walk_gui_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(optimal_walk_gui_genpy)
add_dependencies(optimal_walk_gui_genpy optimal_walk_gui_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS optimal_walk_gui_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/optimal_walk_gui)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/optimal_walk_gui
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(optimal_walk_gui_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(optimal_walk_gui_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/optimal_walk_gui)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/optimal_walk_gui
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(optimal_walk_gui_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(optimal_walk_gui_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/optimal_walk_gui)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/optimal_walk_gui
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(optimal_walk_gui_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(optimal_walk_gui_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/optimal_walk_gui)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/optimal_walk_gui
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(optimal_walk_gui_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(optimal_walk_gui_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/optimal_walk_gui)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/optimal_walk_gui\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/optimal_walk_gui
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(optimal_walk_gui_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(optimal_walk_gui_generate_messages_py std_msgs_generate_messages_py)
endif()
