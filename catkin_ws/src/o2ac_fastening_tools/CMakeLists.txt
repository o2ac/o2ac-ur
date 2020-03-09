################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(o2ac_fastening_tools)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
  message_generation
  genmsg
  actionlib
  actionlib_msgs
  roscpp
)

################################################################################
# Declare ROS messages, services and actions
################################################################################

add_service_files(
  FILES
  DynamixelWriteCommand.srv
  DynamixelReadState.srv
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs 
)

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp dynamixel_workbench_msgs dynamixel_workbench_toolbox message_runtime actionlib_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dynamixel_controller src/dynamixel_controller.cpp)
add_dependencies(dynamixel_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_controller ${catkin_LIBRARIES})

