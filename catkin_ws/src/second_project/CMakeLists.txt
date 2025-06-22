cmake_minimum_required(VERSION 2.8.3)
project(second_project)

## Find catkin macros and libraries first
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib
  move_base_msgs
  tf
  tf2_ros
  sensor_msgs
  message_filters
)

## Declare catkin package with dependencies
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs actionlib move_base_msgs tf tf2_ros message_filters sensor_msgs
)

## Include headers for compilation
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare executable and link libraries
add_executable(goal_setter src/goal_setter.cpp)
add_executable(scan_combiner_node src/scan_combiner_node.cpp)
add_executable(odom_to_tf src/odom_to_tf.cpp)
target_link_libraries(goal_setter ${catkin_LIBRARIES})
target_link_libraries(odom_to_tf
  ${catkin_LIBRARIES}
)
target_link_libraries(scan_combiner_node
  ${catkin_LIBRARIES}
)
