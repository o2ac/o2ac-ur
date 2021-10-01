cmake_minimum_required(VERSION 2.8.3)
project(fast_downward_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_srvs
    std_msgs
)

# Declare the message files to be built
add_message_files(DIRECTORY msg)

add_service_files(DIRECTORY srv)

# Generate actions in the 'action' folder
# add_action_files(DIRECTORY action)

# # Generate added messages and services with any dependencies listed here

# Actually generate the language-specific message and service files
generate_messages(
  DEPENDENCIES
  std_srvs
  std_msgs
)


catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
)
