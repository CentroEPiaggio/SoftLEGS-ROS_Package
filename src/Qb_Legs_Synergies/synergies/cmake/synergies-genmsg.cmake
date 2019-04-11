# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "synergies: 4 messages, 3 services")

set(MSG_I_FLAGS "-Isynergies:/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(synergies_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/bias.msg" NAME_WE)
add_custom_target(_synergies_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "synergies" "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/bias.msg" ""
)

get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/traj_state.msg" NAME_WE)
add_custom_target(_synergies_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "synergies" "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/traj_state.msg" ""
)

get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/data_corr_AT.msg" NAME_WE)
add_custom_target(_synergies_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "synergies" "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/data_corr_AT.msg" ""
)

get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/speed_srv.srv" NAME_WE)
add_custom_target(_synergies_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "synergies" "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/speed_srv.srv" ""
)

get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/warm_start_srv.srv" NAME_WE)
add_custom_target(_synergies_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "synergies" "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/warm_start_srv.srv" ""
)

get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/gain_bias_err.msg" NAME_WE)
add_custom_target(_synergies_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "synergies" "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/gain_bias_err.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/warm_stop_srv.srv" NAME_WE)
add_custom_target(_synergies_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "synergies" "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/warm_stop_srv.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/bias.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/synergies
)
_generate_msg_cpp(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/traj_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/synergies
)
_generate_msg_cpp(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/gain_bias_err.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/synergies
)
_generate_msg_cpp(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/data_corr_AT.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/synergies
)

### Generating Services
_generate_srv_cpp(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/warm_stop_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/synergies
)
_generate_srv_cpp(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/speed_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/synergies
)
_generate_srv_cpp(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/warm_start_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/synergies
)

### Generating Module File
_generate_module_cpp(synergies
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/synergies
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(synergies_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(synergies_generate_messages synergies_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/bias.msg" NAME_WE)
add_dependencies(synergies_generate_messages_cpp _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/traj_state.msg" NAME_WE)
add_dependencies(synergies_generate_messages_cpp _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/data_corr_AT.msg" NAME_WE)
add_dependencies(synergies_generate_messages_cpp _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/speed_srv.srv" NAME_WE)
add_dependencies(synergies_generate_messages_cpp _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/warm_start_srv.srv" NAME_WE)
add_dependencies(synergies_generate_messages_cpp _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/gain_bias_err.msg" NAME_WE)
add_dependencies(synergies_generate_messages_cpp _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/warm_stop_srv.srv" NAME_WE)
add_dependencies(synergies_generate_messages_cpp _synergies_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(synergies_gencpp)
add_dependencies(synergies_gencpp synergies_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS synergies_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/bias.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/synergies
)
_generate_msg_eus(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/traj_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/synergies
)
_generate_msg_eus(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/gain_bias_err.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/synergies
)
_generate_msg_eus(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/data_corr_AT.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/synergies
)

### Generating Services
_generate_srv_eus(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/warm_stop_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/synergies
)
_generate_srv_eus(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/speed_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/synergies
)
_generate_srv_eus(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/warm_start_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/synergies
)

### Generating Module File
_generate_module_eus(synergies
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/synergies
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(synergies_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(synergies_generate_messages synergies_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/bias.msg" NAME_WE)
add_dependencies(synergies_generate_messages_eus _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/traj_state.msg" NAME_WE)
add_dependencies(synergies_generate_messages_eus _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/data_corr_AT.msg" NAME_WE)
add_dependencies(synergies_generate_messages_eus _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/speed_srv.srv" NAME_WE)
add_dependencies(synergies_generate_messages_eus _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/warm_start_srv.srv" NAME_WE)
add_dependencies(synergies_generate_messages_eus _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/gain_bias_err.msg" NAME_WE)
add_dependencies(synergies_generate_messages_eus _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/warm_stop_srv.srv" NAME_WE)
add_dependencies(synergies_generate_messages_eus _synergies_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(synergies_geneus)
add_dependencies(synergies_geneus synergies_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS synergies_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/bias.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/synergies
)
_generate_msg_lisp(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/traj_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/synergies
)
_generate_msg_lisp(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/gain_bias_err.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/synergies
)
_generate_msg_lisp(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/data_corr_AT.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/synergies
)

### Generating Services
_generate_srv_lisp(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/warm_stop_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/synergies
)
_generate_srv_lisp(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/speed_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/synergies
)
_generate_srv_lisp(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/warm_start_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/synergies
)

### Generating Module File
_generate_module_lisp(synergies
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/synergies
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(synergies_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(synergies_generate_messages synergies_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/bias.msg" NAME_WE)
add_dependencies(synergies_generate_messages_lisp _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/traj_state.msg" NAME_WE)
add_dependencies(synergies_generate_messages_lisp _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/data_corr_AT.msg" NAME_WE)
add_dependencies(synergies_generate_messages_lisp _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/speed_srv.srv" NAME_WE)
add_dependencies(synergies_generate_messages_lisp _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/warm_start_srv.srv" NAME_WE)
add_dependencies(synergies_generate_messages_lisp _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/gain_bias_err.msg" NAME_WE)
add_dependencies(synergies_generate_messages_lisp _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/warm_stop_srv.srv" NAME_WE)
add_dependencies(synergies_generate_messages_lisp _synergies_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(synergies_genlisp)
add_dependencies(synergies_genlisp synergies_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS synergies_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/bias.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/synergies
)
_generate_msg_nodejs(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/traj_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/synergies
)
_generate_msg_nodejs(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/gain_bias_err.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/synergies
)
_generate_msg_nodejs(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/data_corr_AT.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/synergies
)

### Generating Services
_generate_srv_nodejs(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/warm_stop_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/synergies
)
_generate_srv_nodejs(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/speed_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/synergies
)
_generate_srv_nodejs(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/warm_start_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/synergies
)

### Generating Module File
_generate_module_nodejs(synergies
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/synergies
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(synergies_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(synergies_generate_messages synergies_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/bias.msg" NAME_WE)
add_dependencies(synergies_generate_messages_nodejs _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/traj_state.msg" NAME_WE)
add_dependencies(synergies_generate_messages_nodejs _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/data_corr_AT.msg" NAME_WE)
add_dependencies(synergies_generate_messages_nodejs _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/speed_srv.srv" NAME_WE)
add_dependencies(synergies_generate_messages_nodejs _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/warm_start_srv.srv" NAME_WE)
add_dependencies(synergies_generate_messages_nodejs _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/gain_bias_err.msg" NAME_WE)
add_dependencies(synergies_generate_messages_nodejs _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/warm_stop_srv.srv" NAME_WE)
add_dependencies(synergies_generate_messages_nodejs _synergies_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(synergies_gennodejs)
add_dependencies(synergies_gennodejs synergies_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS synergies_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/bias.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/synergies
)
_generate_msg_py(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/traj_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/synergies
)
_generate_msg_py(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/gain_bias_err.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/synergies
)
_generate_msg_py(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/data_corr_AT.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/synergies
)

### Generating Services
_generate_srv_py(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/warm_stop_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/synergies
)
_generate_srv_py(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/speed_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/synergies
)
_generate_srv_py(synergies
  "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/warm_start_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/synergies
)

### Generating Module File
_generate_module_py(synergies
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/synergies
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(synergies_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(synergies_generate_messages synergies_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/bias.msg" NAME_WE)
add_dependencies(synergies_generate_messages_py _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/traj_state.msg" NAME_WE)
add_dependencies(synergies_generate_messages_py _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/data_corr_AT.msg" NAME_WE)
add_dependencies(synergies_generate_messages_py _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/speed_srv.srv" NAME_WE)
add_dependencies(synergies_generate_messages_py _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/warm_start_srv.srv" NAME_WE)
add_dependencies(synergies_generate_messages_py _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/msg/gain_bias_err.msg" NAME_WE)
add_dependencies(synergies_generate_messages_py _synergies_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/synergies/srv/warm_stop_srv.srv" NAME_WE)
add_dependencies(synergies_generate_messages_py _synergies_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(synergies_genpy)
add_dependencies(synergies_genpy synergies_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS synergies_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/synergies)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/synergies
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(synergies_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(synergies_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/synergies)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/synergies
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(synergies_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(synergies_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/synergies)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/synergies
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(synergies_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(synergies_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/synergies)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/synergies
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(synergies_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(synergies_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/synergies)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/synergies\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/synergies
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(synergies_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(synergies_generate_messages_py std_msgs_generate_messages_py)
endif()
