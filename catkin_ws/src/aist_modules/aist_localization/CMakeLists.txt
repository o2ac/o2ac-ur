cmake_minimum_required(VERSION 2.8.3)
project(aist_localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-w)
add_compile_options(-std=c++14)
add_compile_options(-fpermissive)
add_compile_options(-pthread)

find_package(
  PhoLocalization REQUIRED
  CONFIG PATHS "$ENV{PHO_LOCALIZATION_PATH}"
)

find_package(
  catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    roslib
    genmsg
    message_generation
    actionlib_msgs
    actionlib
    std_msgs
    sensor_msgs
    geometry_msgs
    ddynamic_reconfigure
    tf
    aist_depth_filter
)

find_package(OpenCV REQUIRED)

catkin_python_setup()

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PHOLOCALIZATION_INCLUDE_DIRS}
)

add_action_files(FILES Localize.action)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    actionlib
    geometry_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    sensor_msgs
    pluginlib
    nodelet
)

add_executable(
  ${PROJECT_NAME}
  src/main.cpp
  src/Localization.cpp
)

add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_gencpp
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${roscpp_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PHOLOCALIZATION_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_library(
  ${PROJECT_NAME}_nodelet
  src/nodelet.cpp
  src/Localization.cpp
)

add_dependencies(
  ${PROJECT_NAME}_nodelet
  ${PROJECT_NAME}_gencpp
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}_nodelet
  ${roscpp_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PHOLOCALIZATION_LIBRARIES}
)

#############
## Install ##
#############
install(
  TARGETS
    ${PROJECT_NAME} ${PROJECT_NAME}_nodelet
  ARCHIVE
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
