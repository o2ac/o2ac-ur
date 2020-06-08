cmake_minimum_required(VERSION 2.8.3)
project(o2ac_examples)

## Add support for C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  tf_conversions
  moveit_core 
  moveit_commander
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
  rosparam_shortcuts
  robotiq_msgs
  o2ac_skills
  o2ac_routines
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   DIRECTORY srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
#   geometry_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES o2ac_examples
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/o2ac_examples.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable

add_executable(example_loop src/example_loop.cpp)
add_dependencies(example_loop 
	${${PROJECT_NAME}_EXPORTED_TARGETS}
  	${catkin_EXPORTED_TARGETS})
target_link_libraries(example_loop ${catkin_LIBRARIES})

add_executable(example_tf src/example_tf.cpp)
add_dependencies(example_tf 
	${${PROJECT_NAME}_EXPORTED_TARGETS}
  	${catkin_EXPORTED_TARGETS})
target_link_libraries(example_tf ${catkin_LIBRARIES})

add_executable(grasp_example src/grasp_example.cpp)
add_dependencies(grasp_example 
	${${PROJECT_NAME}_EXPORTED_TARGETS}
  	${catkin_EXPORTED_TARGETS})
target_link_libraries(grasp_example ${catkin_LIBRARIES})

add_executable(action_example src/action_example.cpp)
add_dependencies(action_example 
	${${PROJECT_NAME}_EXPORTED_TARGETS}
  	${catkin_EXPORTED_TARGETS})
target_link_libraries(action_example ${catkin_LIBRARIES})

add_library(o2ac_lib src/o2ac_pick_place_task.cpp)
set_target_properties(o2ac_lib PROPERTIES OUTPUT_NAME o2ac_demo_pick_place)
add_dependencies(o2ac_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(mtc_pick_place_demo src/mtc_pick_place_demo.cpp)
target_link_libraries(mtc_pick_place_demo
  o2ac_lib
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/send_program_to_ur.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_o2ac_examples.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

