#pragma once

#include <memory>
#include <map>
#include <string>
#include <vector>

#include "exploration_swarm/ego_pointcloud_map_adapter.hpp"
#include "exploration_swarm/exploration_common.hpp"
#include "exploration_swarm/exploration_visualizer.hpp"
#include "exploration_swarm/frontier_detector.hpp"
#include "exploration_swarm/hgrid_allocator.hpp"
#include "exploration_swarm/racer_map_visualizer.hpp"
#include "exploration_swarm/role_assigner.hpp"
#include "exploration_swarm/viewpoint_sampler.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace exploration_swarm
{

class ExplorationGoalManager
{
public:
  explicit ExplorationGoalManager(rclcpp::Node & node);

  const ExplorationParameters & parameters() const;

private:
  void declareParameters();
  void readParameters();
  void createGoalPublishers();
  void createOdometrySubscriptions();
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onOdometry(int drone_id, const nav_msgs::msg::Odometry::SharedPtr msg);
  void processLatestMap();
  std::vector<Viewpoint> createFallbackViewpoints(
    const std::vector<FrontierCluster> & clusters) const;
  void updateFrontierGoals(
    const std::vector<FrontierCluster> & frontiers,
    const std::vector<Viewpoint> & viewpoints,
    const std::map<int, std::vector<int>> & hgrid_assignment);
  bool shouldSelectNewGoal(int drone_id) const;
  bool isCurrentGoalReached(int drone_id) const;
  bool isCurrentGoalTimedOut(int drone_id) const;
  bool isGoalValid(const geometry_msgs::msg::PoseStamped & goal) const;
  bool worldToMap(double x, double y, int & mx, int & my) const;
  bool isMapCellFree(int mx, int my) const;
  bool isPositionSafe(const Eigen::Vector3d & position) const;
  bool isSeparatedFromAssignedGoals(
    const Viewpoint & viewpoint,
    const std::vector<geometry_msgs::msg::PoseStamped> & assigned_goals) const;
  bool isViewpointInAssignedHGrid(
    const Viewpoint & viewpoint,
    const std::vector<int> & assigned_grid_ids) const;
  double scoreViewpoint(
    int drone_id,
    const Viewpoint & viewpoint,
    int max_visible_num,
    const std::vector<geometry_msgs::msg::PoseStamped> & assigned_goals) const;
  double scoreCollectorViewpoint(
    int drone_id,
    const Viewpoint & viewpoint,
    const std::vector<geometry_msgs::msg::PoseStamped> & assigned_goals) const;
  geometry_msgs::msg::PoseStamped viewpointToGoal(const Viewpoint & viewpoint) const;
  double currentYaw(int drone_id) const;
  static double normalizeAngle(double angle);
  void onTimer();

  rclcpp::Node & node_;
  ExplorationParameters params_;
  FrontierDetector frontier_detector_;
  ViewpointSampler viewpoint_sampler_;
  ExplorationVisualizer visualizer_;
  RacerMapVisualizer racer_map_visualizer_;
  HGridAllocator hgrid_allocator_;
  RoleAssigner role_assigner_;
  std::unique_ptr<EgoPointCloudMapAdapter> ego_map_adapter_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr explored_area_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_area_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_area_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr explored_area_3d_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_area_3d_pub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  std::vector<nav_msgs::msg::Odometry::SharedPtr> latest_odoms_;
  nav_msgs::msg::OccupancyGrid latest_map_;
  bool has_map_{false};
  std::vector<geometry_msgs::msg::PoseStamped> current_goals_;
  std::vector<geometry_msgs::msg::PoseStamped> previous_goals_;
  std::vector<bool> has_current_goals_;
  std::vector<bool> has_previous_goals_;
  std::vector<rclcpp::Time> current_goal_stamps_;
  std::vector<int> previous_grid_ids_;
  std::vector<Role> current_roles_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> goal_publishers_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace exploration_swarm
