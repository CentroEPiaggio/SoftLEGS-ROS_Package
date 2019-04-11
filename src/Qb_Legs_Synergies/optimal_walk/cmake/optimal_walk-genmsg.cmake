# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "optimal_walk: 1 messages, 3 services")

set(MSG_I_FLAGS "-Ioptimal_walk:/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators

add_custom_target(optimal_walk_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk/srv/sim_start_srv.srv" NAME_WE)
add_custom_target(_optimal_walk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "optimal_walk" "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk/srv/sim_start_srv.srv" ""
)

get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk/msg/dataset_name.msg" NAME_WE)
add_custom_target(_optimal_walk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "optimal_walk" "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk/msg/dataset_name.msg" ""
)

get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk/srv/num_sim_srv.srv" NAME_WE)
add_custom_target(_optimal_walk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "optimal_walk" "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk/srv/num_sim_srv.srv" ""
)

get_filename_component(_filename "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk/srv/sim_stop_srv.srv" NAME_WE)
add_custom_target(_optimal_walk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "optimal_walk" "/home/riccardo/catkin_ws/src/Qb_Legs_Synergies/optimal_walk/srv/sim_stop_srv.srv" ""
)

#
#  langs = 
#


