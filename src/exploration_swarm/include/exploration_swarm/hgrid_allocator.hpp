#pragma once

#include <map>
#include <vector>

#include "Eigen/Core"
#include "exploration_swarm/exploration_common.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace exploration_swarm
{

struct GridCellInfo
{
  int id{0};
  int level{0};
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double size{0.0};
  int unknown_num{0};
  int frontier_num{0};
  int free_num{0};
  bool active{false};
  std::vector<int> contained_frontier_ids;
};

class HGridAllocator
{
public:
  HGridAllocator() = default;

  void setParameters(const PartitioningParameters & params);
  void setMap(const nav_msgs::msg::OccupancyGrid & map);
  void update(const std::vector<FrontierCluster> & frontiers);

  std::vector<int> getActiveGridIds() const;
  const std::vector<GridCellInfo> & getGridCells() const;

  std::map<int, std::vector<int>> assignGridsToDrones(
    const std::vector<Eigen::Vector3d> & drone_positions,
    const std::vector<Eigen::Vector3d> & drone_velocities,
    const std::vector<int> & previous_grid_ids);

  std::vector<int> getFrontiersInAssignedGrids(
    int drone_id,
    const std::map<int, std::vector<int>> & assignment) const;

private:
  void buildGrid();
  void countGridInformation(const std::vector<FrontierCluster> & frontiers);
  double costDroneToGrid(
    const Eigen::Vector3d & drone_pos,
    const GridCellInfo & grid,
    int previous_grid_id) const;
  bool pointInGrid(const Eigen::Vector3d & point, const GridCellInfo & grid) const;

  nav_msgs::msg::OccupancyGrid map_;
  PartitioningParameters params_;
  std::vector<GridCellInfo> grids_;
  bool has_map_{false};
};

}  // namespace exploration_swarm
