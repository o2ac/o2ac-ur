################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(o2ac_pose_distribution_updater)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  actionlib
  actionlib_msgs
  roscpp
  o2ac_msgs
)

find_package(Eigen3 REQUIRED)
find_package(fcl REQUIRED)
find_package(PCL 1.8 REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################


################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS roscpp actionlib_msgs std_msgs geometry_msgs o2ac_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}  
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(update_distribution_action_server src/action_server.cpp)
add_dependencies(update_distribution_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(update_distribution_action_server fcl ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_client_for_place src/test/test_client_for_place.cpp)
add_dependencies(test_client_for_place ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_client_for_place fcl ${catkin_LIBRARIES} ${PCL_LIBRARIES})
