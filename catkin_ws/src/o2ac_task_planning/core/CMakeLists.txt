cmake_minimum_required(VERSION 2.8.12)
project(o2ac_task_planning_core)

add_compile_options(-std=c++14)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
	roscpp
	moveit_core
	moveit_task_constructor_core
  moveit_ros_planning
	moveit_ros_planning_interface
	rosparam_shortcuts
  moveit_task_constructor_msgs
  o2ac_task_planning_msgs
  visualization_msgs
  rviz_marker_tools
)

catkin_package(
  LIBRARIES
		${PROJECT_NAME}_stages
	INCLUDE_DIRS
		include
	CATKIN_DEPENDS
		geometry_msgs
		moveit_core
		moveit_task_constructor_msgs
    o2ac_task_planning_msgs
		visualization_msgs
)

# if ("${CMAKE_VERSION}" VERSION_LESS "3.1")
# 	add_compile_options(-std=c++14)
# else ()
# 	set(CMAKE_CXX_STANDARD 14)
# endif ()

# # check for MOVEIT_MASTER
# include(CheckIncludeFileCXX)
# set(CMAKE_REQUIRED_INCLUDES ${moveit_core_INCLUDE_DIRS})
# set(CMAKE_REQUIRED_FLAGS "-std=c++11")
# CHECK_INCLUDE_FILE_CXX(moveit/collision_detection/collision_env.h MOVEIT_MASTER)
# if(NOT MOVEIT_MASTER)
#   set(MOVEIT_MASTER 0)
# endif()
# add_definitions(-DMOVEIT_MASTER=${MOVEIT_MASTER})

set(PROJECT_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)
