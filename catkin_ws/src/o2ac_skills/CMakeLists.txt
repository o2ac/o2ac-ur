cmake_minimum_required(VERSION 2.8.3)
project(o2ac_skills)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
	rospy
	roscpp
	std_msgs
	control_msgs
	robotiq_msgs
	o2ac_msgs
	std_srvs
	moveit_core
	moveit_commander
	moveit_ros_planning_interface
	moveit_ros_planning
	tf
	ur_robot_driver
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES o2ac_skills
 CATKIN_DEPENDS 	std_msgs control_msgs robotiq_msgs o2ac_msgs std_srvs moveit_core moveit_commander moveit_ros_planning_interface moveit_ros_planning tf ur_robot_driver
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(o2ac_skills
  src/o2ac_skill_server.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Declare a C++ executable
# With catkin_make all packages are built within a single CMake context
# The recommended prefix ensures that target names across packages don't collide
add_executable(o2ac_skill_server src/o2ac_skill_server.cpp)
add_dependencies(o2ac_skill_server 
	${${PROJECT_NAME}_EXPORTED_TARGETS}
  	${catkin_EXPORTED_TARGETS})
target_link_libraries(o2ac_skill_server ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/ur_program_relay.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
