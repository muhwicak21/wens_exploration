#pragma once

#include <vector>

#include "exploration_swarm/exploration_common.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace exploration_swarm
{

class FrontierDetector
{
public:
  FrontierDetector() = default;

  void setParameters(const FrontierParameters & params, double default_goal_z);
  void setMap(const nav_msgs::msg::OccupancyGrid & map);
  std::vector<FrontierCluster> detectFrontiers();

private:
  bool isInside(int mx, int my) const;
  int toIndex(int mx, int my) const;
  Eigen::Vector3d mapToWorld(int mx, int my) const;
  bool isKnownFree(int mx, int my) const;
  bool isUnknown(int mx, int my) const;
  bool isOccupied(int mx, int my) const;
  bool isFrontierCell(int mx, int my) const;
  void expandFrontierBFS(int start_x, int start_y, FrontierCluster & cluster);
  void computeFrontierInfo(FrontierCluster & cluster);
  void downsampleFrontier(FrontierCluster & cluster);

  nav_msgs::msg::OccupancyGrid map_;
  FrontierParameters params_;
  double default_goal_z_{1.0};
  bool has_map_{false};
  std::vector<bool> visited_;
};

}  // namespace exploration_swarm
