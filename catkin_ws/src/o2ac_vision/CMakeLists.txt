cmake_minimum_required(VERSION 2.8.3)
project(o2ac_vision)

## Add support for C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  tf_conversions
  o2ac_routines
  actionlib_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

# catkin_install_python(PROGRAMS scripts/action_example.py
#                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############
