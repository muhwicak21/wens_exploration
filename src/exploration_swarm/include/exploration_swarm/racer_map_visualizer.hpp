#pragma once

#include <vector>

#include "Eigen/Core"
#include "exploration_swarm/exploration_common.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace exploration_swarm
{

class RacerMapVisualizer
{
public:
  void setParameters(
    const RacerVisualizerParameters & params,
    const std::string & frame_id);
  void setStamp(const rclcpp::Time & stamp);
  void setMap(const nav_msgs::msg::OccupancyGrid & map);

  sensor_msgs::msg::PointCloud2 buildExploredCloud() const;
  sensor_msgs::msg::PointCloud2 buildOccupiedCloud() const;
  sensor_msgs::msg::PointCloud2 buildUnknownCloud() const;

private:
  bool isKnown(int value) const;
  bool isFree(int value) const;
  bool isOccupied(int value) const;
  bool isUnknown(int value) const;
  Eigen::Vector3d gridToWorld(int mx, int my, double z) const;
  sensor_msgs::msg::PointCloud2 buildCloudFromCells(
    const std::vector<Eigen::Vector3d> & points) const;

  nav_msgs::msg::OccupancyGrid map_;
  RacerVisualizerParameters params_;
  std::string frame_id_{"world"};
  rclcpp::Time stamp_{0, 0, RCL_ROS_TIME};
  bool has_map_{false};
};

}  // namespace exploration_swarm
