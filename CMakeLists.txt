cmake_minimum_required(VERSION 2.8.12)
project(pr2_sensor_grasping)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  pr2_gripper_sensor_action
  roscpp
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gripper_sensor_node src/gripper_sensor_node.cpp)
add_dependencies(gripper_sensor_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(gripper_sensor_node ${catkin_LIBRARIES})

add_executable(force_control_gripper_node src/force_control_gripper_node.cpp)
add_dependencies(force_control_gripper_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(force_control_gripper_node ${catkin_LIBRARIES})