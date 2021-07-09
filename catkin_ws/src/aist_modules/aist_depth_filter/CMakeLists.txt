cmake_minimum_required(VERSION 2.8.3)
project(aist_depth_filter)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-w)
add_compile_options(-std=c++14)
add_compile_options(-fpermissive)
add_compile_options(-pthread)

find_package(PkgConfig)
pkg_check_modules(LIBTIFF REQUIRED libtiff-4)

find_package(
  catkin REQUIRED
  COMPONENTS
    ddynamic_reconfigure
    sensor_msgs
    image_transport
    message_generation
    actionlib_msgs
    actionlib
    rospy
)

find_package(OpenCV REQUIRED)

catkin_python_setup()

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${LIBTIFF_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  src
)

add_message_files(FILES FileInfo.msg Plane.msg PlaneStamped.msg)
add_action_files(FILES DetectPlane.action)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    actionlib
    sensor_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    message_runtime
    sensor_msgs
    pluginlib
    nodelet
)

add_executable(
  ${PROJECT_NAME}
  src/main.cpp
  src/DepthFilter.cpp
  src/tiff.cpp
  src/ply.cpp
  src/oply/OrderedPly.cpp
  src/rply/rply.c
)

add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_gencpp
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${LIBTIFF_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_library(
  ${PROJECT_NAME}_nodelet
  src/nodelet.cpp
  src/DepthFilter.cpp
  src/tiff.cpp
  src/ply.cpp
  src/oply/OrderedPly.cpp
  src/rply/rply.c
)

add_dependencies(
  ${PROJECT_NAME}_nodelet
  ${PROJECT_NAME}_gencpp
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${LIBTIFF_LIBRARIES}
  ${OpenCV_LIBRARIES}
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
