cmake_minimum_required(VERSION 2.8.3)
project(robot_navigation_state)

set( ROBOT_NAVIGATION_STATE_PKG_DEPENDS
  roscpp
  tf
  geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS ${ROBOT_NAVIGATION_STATE_PKG_DEPENDS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_navigation_state
  CATKIN_DEPENDS ${ROBOT_NAVIGATION_STATE_PKG_DEPENDS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(robot_navigation_state
  src/robot_navigation_state.cpp
)

target_link_libraries(robot_navigation_state
  ${catkin_LIBRARIES}
)

install(TARGETS robot_navigation_state
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
