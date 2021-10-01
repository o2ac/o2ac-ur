cmake_minimum_required(VERSION 2.8.3)
project(aist_new_localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-w)
add_compile_options(-std=c++14)
add_compile_options(-fpermissive)
add_compile_options(-pthread)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED 1.8 REQUIRED COMPONENTS common io)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    roslib
    image_transport
    geometry_msgs
    genmsg
    message_generation
    actionlib_msgs
    actionlib
    ddynamic_reconfigure
    pcl_ros
    pcl_conversions
    cv_bridge
    tf
    aist_depth_filter
)

catkin_python_setup()

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
#  ${PCL_INCLUDE_DIRS}
)

add_action_files(FILES Localize.action)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    actionlib
    geometry_msgs
    std_msgs
    aist_depth_filter
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    geometry_msgs
    pluginlib
    nodelet
)

add_executable(${PROJECT_NAME}
  src/main.cpp src/Localization.cpp
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
  src/nodelet.cpp src/Localization.cpp
)

add_dependencies(${PROJECT_NAME}_nodelet
  ${PROJECT_NAME}_gencpp
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(mesh_sampler
  src/mesh_sampler.cpp
)

add_dependencies(mesh_sampler
  ${PROJECT_NAME}_gencpp
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(mesh_sampler
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
)

#############
## Install ##
#############
install(
  TARGETS
    ${PROJECT_NAME} ${PROJECT_NAME}_nodelet mesh_sampler
  ARCHIVE
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
