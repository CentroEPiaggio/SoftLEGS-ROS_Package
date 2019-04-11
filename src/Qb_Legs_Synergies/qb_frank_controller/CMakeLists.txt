cmake_minimum_required(VERSION 2.8.3)
project(qb_frank_controller)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp roslib)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(qb_frank_controller_test_right src/qb_frank_controller_test_right.cpp)
add_dependencies(qb_frank_controller_test_right ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(qb_frank_controller_test_right ${catkin_LIBRARIES})

add_executable(qb_frank_controller_test_left src/qb_frank_controller_test_left.cpp)
add_dependencies(qb_frank_controller_test_left ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(qb_frank_controller_test_left ${catkin_LIBRARIES})

add_executable(qb_frank_controller_test_full src/qb_frank_controller_test_full.cpp)
add_dependencies(qb_frank_controller_test_full ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(qb_frank_controller_test_full ${catkin_LIBRARIES})

add_executable(qb_frank_controller_full src/qb_frank_controller_full.cpp)
add_dependencies(qb_frank_controller_full ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(qb_frank_controller_full ${catkin_LIBRARIES})

add_executable(qb_frank_controller_test_pub src/qb_frank_controller_test_pub.cpp)
add_dependencies(qb_frank_controller_test_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(qb_frank_controller_test_pub ${catkin_LIBRARIES})