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
  sensor_msgs
  cv_bridge
  actionlib
  actionlib_msgs
  roscpp
  eigen_conversions
  o2ac_msgs
  tf2_ros
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(PkgConfig)

pkg_check_modules(FCL REQUIRED fcl)

################################################################################
# Declare ROS messages, services and actions
################################################################################


################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp actionlib_msgs std_msgs geometry_msgs o2ac_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
)

add_library(ros_converters src/ros_converters.cpp)
target_link_libraries(ros_converters ${catkin_LIBRARIES})
add_library(read_stl src/read_stl.cpp)
add_library(estimator src/estimator.cpp src/place_action_helpers.cpp src/grasp_action_helpers.cpp)
target_link_libraries(estimator ${FCL_LIBRARIES} ${OpenCV_LIBRARIES} )
add_library(distribution_conversions src/distribution_conversions.cpp)

add_executable(update_distribution_action_server src/action_server.cpp src/ros_converted_estimator.cpp src/pose_belief_visualizer.cpp)
add_dependencies(update_distribution_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(update_distribution_action_server estimator ros_converters distribution_conversions)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_client test/unit_test.test src/test/test_client.cpp src/test/touch_test.cpp src/test/look_test.cpp src/test/place_test.cpp src/test/grasp_test.cpp)
  target_link_libraries(test_client ros_converters ${FCL_LIBRARIES} ${OpenCV_LIBRARIES} read_stl distribution_conversions)

  add_executable(visualize_test src/test/visuzalize_test.cpp)
  add_dependencies(visualize_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(visualize_test ros_converters read_stl)

endif()
