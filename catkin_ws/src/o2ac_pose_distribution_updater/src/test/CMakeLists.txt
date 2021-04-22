cmake_minimum_required(VERSION 2.8.3)
project(in_hand_pose_estimator)

find_package(fcl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

include_directories(
  ${FCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}  
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

#add_executable(test test_estimator_touch.cpp)
#add_executable(gen generate_test_case.cpp)
#target_link_libraries(test fcl ${PCL_LIBRARIES})
#target_link_libraries(gen fcl ${PCL_LIBRARIES})

add_executable(test_place test_for_place_poly.cpp)
target_link_libraries(test_place fcl ${PCL_LIBRARIES})
