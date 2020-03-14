cmake_minimum_required(VERSION 2.8.12)
project(aist_phoxi_camera)

find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    roslib
    rospy
    image_transport
    std_msgs
    sensor_msgs
    ddynamic_reconfigure
    tf
    pluginlib
    nodelet
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PHOXI_INCLUDE_DIRS}
)

add_definitions(-DPHO_SOFTWARE_VERSION_MAJOR=${PHO_SOFTWARE_VERSION_MAJOR}
	        -DPHO_SOFTWARE_VERSION_MINOR=${PHO_SOFTWARE_VERSION_MINOR})

add_message_files(
  FILES
    PhoXiSize.msg
)

add_service_files(
  FILES
    SetString.srv
    GetString.srv
    GetStringList.srv
    GetSupportedCapturingModes.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    std_msgs
    sensor_msgs
)

add_compile_options(-w)
add_compile_options(-std=c++14)
add_compile_options(-fpermissive)
add_compile_options(-pthread)

add_executable(
  ${PROJECT_NAME}
  src/Camera.cpp
  src/main.cpp
)

add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_gencpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${PHOXI_LIBRARY}
  ${catkin_LIBRARIES}
)

add_library(
  ${PROJECT_NAME}_nodelet
  src/Camera.cpp
  src/nodelet.cpp
)

add_dependencies(
  ${PROJECT_NAME}_nodelet
  ${PROJECT_NAME}_gencpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}_nodelet
  ${PHOXI_LIBRARY}
  ${catkin_LIBRARIES}
)

install(
  PROGRAMS
    src/aist_phoxi_camera_example.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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

install(
  FILES
    ${PHOXI_LIB_RELEASE_PER_COMPILER}.${PHO_SOFTWARE_VERSION}
  DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)
