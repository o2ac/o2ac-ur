add_library(${PROJECT_NAME}_stages
	${PROJECT_INCLUDE}/stages/dummy.h
	${PROJECT_INCLUDE}/stages/generate_pose.h
	${PROJECT_INCLUDE}/stages/load_grasp_pose.h
	${PROJECT_INCLUDE}/stages/generate_place_pose.h
	${PROJECT_INCLUDE}/stages/generate_handover_pose.h

	dummy.cpp
	generate_pose.cpp
	load_grasp_pose.cpp
	generate_place_pose.cpp
	generate_handover_pose.cpp
)
target_link_libraries(${PROJECT_NAME}_stages ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME}_stages
	PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/stages>
	PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	PUBLIC ${catkin_INCLUDE_DIRS}
)
add_dependencies(${PROJECT_NAME}_stages ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}_stages
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
