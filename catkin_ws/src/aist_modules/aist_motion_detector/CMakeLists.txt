cmake_minimum_required(VERSION 2.8.3)
project(aist_motion_detector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-w)
add_compile_options(-std=c++14)
add_compile_options(-fpermissive)
add_compile_options(-pthread)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

find_package(
  catkin REQUIRED
  COMPONENTS
    roscpp
    roslib
    image_transport
    tf
    sensor_msgs
    geometry_msgs
    genmsg
    message_generation
    actionlib_msgs
    actionlib
    ddynamic_reconfigure
    cv_bridge
)

#catkin_python_setup()

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  src
)

add_action_files(
  FILES FindCabletip.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    actionlib
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    sensor_msgs
    geometry_msgs
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/MotionDetector.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencpp
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_library(${PROJECT_NAME}_nodelet
  src/nodelet.cpp
  src/MotionDetector.cpp
)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_nodelet
  ${PROJECT_NAME}_gencpp
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
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
