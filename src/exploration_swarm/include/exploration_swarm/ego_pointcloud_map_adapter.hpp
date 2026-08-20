#pragma once

#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

#include "exploration_swarm/exploration_common.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace exploration_swarm
{

struct PointCloudMapParameters
{
  std::string cloud_topic{"/map_generator/global_cloud"};
  std::string local_cloud_topic_suffix{"pcl_render_node/cloud"};
  std::string grid_cloud_topic_suffix{"grid/grid_map/occupancy_inflate"};
  std::string occupancy_grid_topic{"/swarm_exploration/occupancy_grid"};
  std::string odom_topic_suffix{"visual_slam/odom"};
  double resolution{0.1};
  double sensing_radius{5.0};
  double occupied_min_z{0.05};
  double occupied_max_z{5.0};
};

class EgoPointCloudMapAdapter
{
public:
  EgoPointCloudMapAdapter(rclcpp::Node & node, const ExplorationParameters & params);

  void start();
  bool hasMap() const;
  const nav_msgs::msg::OccupancyGrid & occupancyGrid() const;
  sensor_msgs::msg::PointCloud2 buildExploredCloud3D(const rclcpp::Time & stamp) const;
  sensor_msgs::msg::PointCloud2 buildOccupiedCloud3D(const rclcpp::Time & stamp) const;

private:
  void onGlobalCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void onLocalCloud(int drone_id, const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void onGridCloud(int drone_id, const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void onOdometry(int drone_id, const nav_msgs::msg::Odometry::SharedPtr msg);
  void initializeGrid(const rclcpp::Time & stamp, const std::string & frame_id);
  void updateGridFromDroneObservation(int drone_id, const rclcpp::Time & stamp);
  bool worldToMap(double x, double y, int & mx, int & my) const;
  int toIndex(int mx, int my) const;
  bool worldToVoxel(double x, double y, double z, int & vx, int & vy, int & vz) const;
  int64_t voxelKey(int vx, int vy, int vz) const;
  sensor_msgs::msg::PointCloud2 buildCloudFromVoxelSet(
    const std::unordered_set<int64_t> & voxels,
    const rclcpp::Time & stamp) const;
  void markKnownFreeAroundDrone(int drone_id);
  void markOccupiedFromLocalCloud(int drone_id);
  void markOccupiedFromGridCloud(int drone_id);
  void accumulateLocalCloud3D(int drone_id);
  void accumulateGridCloud3D(int drone_id);

  rclcpp::Node & node_;
  ExplorationParameters params_;
  PointCloudMapParameters map_params_;
  nav_msgs::msg::OccupancyGrid grid_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_global_cloud_;
  std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> latest_local_clouds_;
  std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> latest_grid_clouds_;
  std::vector<nav_msgs::msg::Odometry::SharedPtr> latest_odoms_;
  std::unordered_set<int64_t> explored_voxels_3d_;
  std::unordered_set<int64_t> occupied_voxels_3d_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> local_cloud_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> grid_cloud_subs_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  bool has_grid_{false};
};

}  // namespace exploration_swarm
