cmake_minimum_required(VERSION 2.8.3)
project(aist_aruco_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-w)
add_compile_options(-std=c++14)
add_compile_options(-fpermissive)
add_compile_options(-pthread)

find_package(catkin REQUIRED
  COMPONENTS
    ddynamic_reconfigure
    geometry_msgs
    visualization_msgs
    image_transport
    cv_bridge
    aruco
    aruco_ros
    aruco_msgs
    tf
)

find_package(OpenCV REQUIRED)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    geometry_msgs
    visualization_msgs
    pluginlib
    nodelet
)

add_executable(aist_aruco_detector
  src/main.cpp
  src/Detector.cpp
)

add_dependencies(aist_aruco_detector
  ${PROJECT_NAME}_gencpp
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(aist_aruco_detector
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_library(aist_aruco_detector_nodelet
  src/nodelet.cpp
  src/Detector.cpp
)

add_dependencies(aist_aruco_detector_nodelet
  ${PROJECT_NAME}_gencpp
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(aist_aruco_detector_nodelet
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############
install(
  TARGETS
    aist_aruco_detector aist_aruco_detector_nodelet
  ARCHIVE
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
