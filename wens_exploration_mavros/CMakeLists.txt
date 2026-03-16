cmake_minimum_required(VERSION 3.0.2)
project(wens_exploration_mavros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  mavros_msgs
  tf2
  tf2_ros
  actionlib
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    mavros_msgs
    tf2
    tf2_ros
    actionlib
    actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Exploration node
add_executable(exploration_node src/exploration_node.cpp)
target_link_libraries(exploration_node ${catkin_LIBRARIES})
add_dependencies(exploration_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Waypoint manager node
add_executable(waypoint_manager_node src/waypoint_manager_node.cpp)
target_link_libraries(waypoint_manager_node ${catkin_LIBRARIES})
add_dependencies(waypoint_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/exploration_node.py
  scripts/waypoint_follower.py
  scripts/offboard_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# Install world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)

# Install include files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
