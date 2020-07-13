cmake_minimum_required(VERSION 2.8.3)
project(rosparam_shortcuts)

# C++ 14
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  eigen_conversions # TODO(davetcoleman) remove this dependency, only for build testing
  roslint
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
  DEPENDS
    EIGEN3
)

include_directories(SYSTEM
  ${EIGEN3_INCLUDE_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/rosparam_shortcuts.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Example executable
add_executable(${PROJECT_NAME}_example src/example.cpp)
set_target_properties(${PROJECT_NAME}_example PROPERTIES OUTPUT_NAME example PREFIX "")
target_link_libraries(${PROJECT_NAME}_example
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Test for correct C++ source code
## To run: catkin bot --make-args roslint
roslint_cpp()

#############
## Install ##
#############

# Mark cpp header files for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark roslaunch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
  PATTERN ".svn" EXCLUDE
)

# Mark config files for installation
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.yaml"
  PATTERN ".svn" EXCLUDE
)
