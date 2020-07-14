cmake_minimum_required(VERSION 2.8.3)
project(aist_visual_tracker)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-w)
add_compile_options(-std=c++14)
add_compile_options(-fpermissive)
add_compile_options(-pthread)

find_package(
  catkin REQUIRED
  COMPONENTS
    ddynamic_reconfigure
    sensor_msgs
    image_transport
    cv_bridge
    message_generation
    rospy
)

find_package(OpenCV REQUIRED)

#catkin_python_setup()

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  src
)

#add_message_files(FILES FileInfo.msg)

#generate_messages(DEPENDENCIES sensor_msgs geometry_msgs)

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
  src/KLTTracker.cpp
  src/klt/convolve.c
  src/klt/error.c
  src/klt/klt.c
  src/klt/klt_util.c
  src/klt/pnmio.c
  src/klt/pyramid.c
  src/klt/selectGoodFeatures.c
  src/klt/storeFeatures.c
  src/klt/trackFeatures.c
  src/klt/writeFeatures.c
)

add_dependencies(
  ${PROJECT_NAME}
#  ${PROJECT_NAME}_gencpp
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_library(
  ${PROJECT_NAME}_nodelet
  src/nodelet.cpp
  src/KLTTracker.cpp
  src/klt/convolve.c
  src/klt/error.c
  src/klt/klt.c
  src/klt/klt_util.c
  src/klt/pnmio.c
  src/klt/pyramid.c
  src/klt/selectGoodFeatures.c
  src/klt/storeFeatures.c
  src/klt/trackFeatures.c
  src/klt/writeFeatures.c
)

add_dependencies(
  ${PROJECT_NAME}_nodelet
#  ${PROJECT_NAME}_gencpp
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
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
