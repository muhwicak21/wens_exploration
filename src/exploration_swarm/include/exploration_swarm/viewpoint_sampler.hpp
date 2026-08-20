#pragma once

#include <vector>

#include "Eigen/Core"
#include "exploration_swarm/exploration_common.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace exploration_swarm
{

struct Viewpoint
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  int visible_num{0};
  int frontier_id{0};
};

class ViewpointSampler
{
public:
  ViewpointSampler() = default;

  void setParameters(
    const FrontierParameters & frontier_params,
    const PerceptionParameters & perception_params);
  void setMap(const nav_msgs::msg::OccupancyGrid & map);
  std::vector<Viewpoint> sampleViewpoints(const FrontierCluster & frontier);

private:
  bool worldToMap(const Eigen::Vector3d & p, int & mx, int & my) const;
  Eigen::Vector3d mapToWorld(int mx, int my) const;
  bool isInsideMap(const Eigen::Vector3d & p) const;
  bool isFree(const Eigen::Vector3d & p) const;
  bool hasMinClearance(const Eigen::Vector3d & p) const;
  bool hasLineOfSight(const Eigen::Vector3d & from, const Eigen::Vector3d & to) const;
  int countVisibleCells(
    const Eigen::Vector3d & viewpoint,
    double yaw,
    const FrontierCluster & frontier) const;

  nav_msgs::msg::OccupancyGrid map_;
  FrontierParameters frontier_params_;
  PerceptionParameters perception_params_;
  bool has_map_{false};
};

}  // namespace exploration_swarm
