cmake_minimum_required(VERSION 2.8.3)
project(aist_handeye_calibration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML-CPP yaml-cpp REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    image_proc
    image_transport
    std_srvs
    std_msgs
    sensor_msgs
    geometry_msgs
    message_generation
    tf
    dynamic_reconfigure
    pcl_ros
    pcl_conversions
)

catkin_python_setup()

add_service_files(
  FILES
    GetSampleList.srv
    ComputeCalibration.srv
    LoadTransform.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    std_msgs
    geometry_msgs
)

## System dependencies are found with CMake's conventions

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  src/calibrator
)

add_executable(calibrator
  src/calibrator/main.cpp
  src/calibrator/Calibrator.cpp
)

add_dependencies(calibrator
  ${PROJECT_NAME}_gencpp
)

target_link_libraries(calibrator
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${YAML-CPP_LIBRARIES}
)

add_executable(quat2rpy
  src/quat2rpy/main.cpp
)

target_link_libraries(quat2rpy
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
)

add_executable(calibrator_test
  src/calibrator_test/main.cpp
)

add_dependencies(calibrator_test
  ${PROJECT_NAME}_gencpp
)

target_link_libraries(calibrator_test
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
)

#############
## Install ##
#############
catkin_install_python(
  PROGRAMS
    scripts/run_calibration.py
    scripts/check_calibration.py
    scripts/publish_calibration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS
    calibrator quat2rpy calibrator_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
