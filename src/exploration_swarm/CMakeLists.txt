cmake_minimum_required(VERSION 3.8)
project(exploration_swarm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

set(DEPENDENCIES
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

add_executable(exploration_goal_dummy src/exploration_goal_dummy.cpp)
ament_target_dependencies(exploration_goal_dummy rclcpp geometry_msgs)

add_executable(swarm_exploration_node
  src/swarm_exploration_node.cpp
  src/ego_pointcloud_map_adapter.cpp
  src/exploration_goal_manager.cpp
  src/exploration_visualizer.cpp
  src/frontier_detector.cpp
  src/racer_map_visualizer.cpp
  src/viewpoint_sampler.cpp
  src/hgrid_allocator.cpp
  src/role_assigner.cpp
)
ament_target_dependencies(swarm_exploration_node ${DEPENDENCIES})
target_link_libraries(swarm_exploration_node Eigen3::Eigen)

install(
  TARGETS exploration_goal_dummy swarm_exploration_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
