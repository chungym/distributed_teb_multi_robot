cmake_minimum_required(VERSION 3.0.2)
project(robot_scan_matcher)

## Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS pluginlib roscpp pcl_ros teb_local_planner)

## Find Boost (headers only)
find_package(Boost REQUIRED)

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/matcher.cfg
)

## Define catkin exports
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_scan_matcher
  CATKIN_DEPENDS pluginlib roscpp pcl_ros teb_local_planner
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## robot_scan_matcher_plugins library
add_library(robot_scan_matcher src/robot_scan_matcher_plugins.cpp)
target_link_libraries(robot_scan_matcher ${catkin_LIBRARIES})

# Dynamic reconfigure: make sure configure headers raare built before any node using them
add_dependencies(robot_scan_matcher ${PROJECT_NAME}_gencfg)

## robot_scan_matcher_node executable
add_executable(standalone_matcher src/robot_scan_matcher_node.cpp)
target_link_libraries(standalone_matcher ${catkin_LIBRARIES})
add_dependencies(standalone_matcher robot_scan_matcher)

## Mark executables and/or libraries for installation
install(TARGETS robot_scan_matcher standalone_matcher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES robot_scan_matcher_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
