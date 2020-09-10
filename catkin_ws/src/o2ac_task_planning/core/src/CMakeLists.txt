add_subdirectory(stages)

add_executable(mtc_modules_server mtc_modules_server.cpp)
target_link_libraries(mtc_modules_server
  ${PROJECT_NAME}_stages
  ${catkin_LIBRARIES})

install(TARGETS mtc_modules_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ${PROJECT_INCLUDE}/stages/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
