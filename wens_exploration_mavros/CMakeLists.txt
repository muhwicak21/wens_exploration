cmake_minimum_required(VERSION 3.0.2)
project(wens_exploration_mavros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  mavros
  mavros_msgs
  octomap_ros
  octomap_msgs
  visualization_msgs
  actionlib
  actionlib_msgs
)

find_package(octomap REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wens_exploration_mavros
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    mavros
    mavros_msgs
    octomap_ros
    octomap_msgs
    visualization_msgs
    actionlib
    actionlib_msgs
  DEPENDS
    OCTOMAP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

# Exploration manager node
add_executable(exploration_manager
  src/exploration_manager.cpp
)
target_link_libraries(exploration_manager
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
add_dependencies(exploration_manager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Frontier detector node
add_executable(frontier_detector
  src/frontier_detector.cpp
)
target_link_libraries(frontier_detector
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
add_dependencies(frontier_detector
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Install targets
install(TARGETS exploration_manager frontier_detector
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch config worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/exploration_controller.py
  scripts/waypoint_navigator.py
  scripts/mission_planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
