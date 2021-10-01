cmake_minimum_required(VERSION 2.8.3)
project(aist_model_spawner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-w)
add_compile_options(-std=c++14)
add_compile_options(-fpermissive)
add_compile_options(-pthread)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    geometry_msgs
    rospy
    tf
    rviz
)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED
  Core
  Widgets
)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

catkin_python_setup()

include_directories(${rviz_INCLUDE_DIRS})

add_service_files(
  FILES
    Add.srv Delete.srv DeleteAll.srv GetList.srv
)

add_message_files(
  FILES
    ModelDescription.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs
    rospy
)

add_library(${PROJECT_NAME}
  src/transient_model_display.cpp
)

add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencpp
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)
