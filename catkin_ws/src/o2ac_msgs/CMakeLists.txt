cmake_minimum_required(VERSION 2.8.3)
project(o2ac_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    cmake_modules
    geometry_msgs
    std_srvs
    std_msgs
    sensor_msgs
    visualization_msgs
    actionlib_msgs
    moveit_msgs
)

# Declare the message files to be built
add_message_files(DIRECTORY msg)

add_service_files(DIRECTORY srv)

# Generate actions in the 'action' folder
add_action_files(DIRECTORY action)
add_action_files(DIRECTORY action/vision)
add_action_files(DIRECTORY action/behavior)

# # Generate added messages and services with any dependencies listed here

# Actually generate the language-specific message and service files
generate_messages(
  DEPENDENCIES
  std_srvs
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  actionlib_msgs
  moveit_msgs
)


catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  actionlib_msgs
  moveit_msgs
)
