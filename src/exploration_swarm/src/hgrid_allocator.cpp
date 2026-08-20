#include "exploration_swarm/hgrid_allocator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace exploration_swarm
{

void HGridAllocator::setParameters(const PartitioningParameters & params)
{
  params_ = params;
  params_.grid_size = std::max(1.0, params_.grid_size);
  params_.min_unknown = std::max(0, params_.min_unknown);
  params_.min_frontier = std::max(0, params_.min_frontier);
  params_.min_free = std::max(0, params_.min_free);
}

void HGridAllocator::setMap(const nav_msgs::msg::OccupancyGrid & map)
{
  map_ = map;
  has_map_ = map_.info.resolution > 0.0 && map_.info.width > 0 && map_.info.height > 0 &&
             !map_.data.empty();
}

void HGridAllocator::update(const std::vector<FrontierCluster> & frontiers)
{
  if (!has_map_)
  {
    grids_.clear();
    return;
  }

  buildGrid();
  countGridInformation(frontiers);
}

std::vector<int> HGridAllocator::getActiveGridIds() const
{
  std::vector<int> active_ids;
  for (const auto & grid : grids_)
  {
    if (grid.active)
    {
      active_ids.push_back(grid.id);
    }
  }
  return active_ids;
}

const std::vector<GridCellInfo> & HGridAllocator::getGridCells() const
{
  return grids_;
}

std::map<int, std::vector<int>> HGridAllocator::assignGridsToDrones(
  const std::vector<Eigen::Vector3d> & drone_positions,
  const std::vector<Eigen::Vector3d> &,
  const std::vector<int> & previous_grid_ids)
{
  std::map<int, std::vector<int>> assignment;
  for (size_t i = 0; i < drone_positions.size(); ++i)
  {
    assignment[static_cast<int>(i)] = {};
  }

  std::vector<const GridCellInfo *> active_grids;
  for (const auto & grid : grids_)
  {
    if (grid.active)
    {
      active_grids.push_back(&grid);
    }
  }

  std::sort(
    active_grids.begin(), active_grids.end(),
    [](const GridCellInfo * lhs, const GridCellInfo * rhs) {
      if (lhs->frontier_num != rhs->frontier_num)
      {
        return lhs->frontier_num > rhs->frontier_num;
      }
      if (lhs->unknown_num != rhs->unknown_num)
      {
        return lhs->unknown_num > rhs->unknown_num;
      }
      return lhs->id < rhs->id;
    });

  std::set<int> assigned_grid_ids;
  for (size_t drone_id = 0; drone_id < drone_positions.size(); ++drone_id)
  {
    int best_grid_id = -1;
    double best_cost = std::numeric_limits<double>::infinity();
    const int previous_grid_id =
      drone_id < previous_grid_ids.size() ? previous_grid_ids[drone_id] : -1;

    for (const auto * grid : active_grids)
    {
      if (assigned_grid_ids.find(grid->id) != assigned_grid_ids.end())
      {
        continue;
      }

      const double cost = costDroneToGrid(
        drone_positions[drone_id], *grid, previous_grid_id);
      if (cost < best_cost)
      {
        best_cost = cost;
        best_grid_id = grid->id;
      }
    }

    if (best_grid_id >= 0)
    {
      assignment[static_cast<int>(drone_id)].push_back(best_grid_id);
      assigned_grid_ids.insert(best_grid_id);
    }
  }

  for (const auto * grid : active_grids)
  {
    if (assigned_grid_ids.find(grid->id) != assigned_grid_ids.end())
    {
      continue;
    }

    int best_drone_id = -1;
    double best_cost = std::numeric_limits<double>::infinity();
    for (size_t drone_id = 0; drone_id < drone_positions.size(); ++drone_id)
    {
      const int previous_grid_id =
        drone_id < previous_grid_ids.size() ? previous_grid_ids[drone_id] : -1;
      const double cost = costDroneToGrid(
        drone_positions[drone_id], *grid, previous_grid_id);
      if (cost < best_cost)
      {
        best_cost = cost;
        best_drone_id = static_cast<int>(drone_id);
      }
    }

    if (best_drone_id >= 0)
    {
      assignment[best_drone_id].push_back(grid->id);
      assigned_grid_ids.insert(grid->id);
    }
  }

  return assignment;
}

std::vector<int> HGridAllocator::getFrontiersInAssignedGrids(
  const int drone_id,
  const std::map<int, std::vector<int>> & assignment) const
{
  std::vector<int> frontier_ids;
  const auto assignment_it = assignment.find(drone_id);
  if (assignment_it == assignment.end())
  {
    return frontier_ids;
  }

  std::set<int> unique_frontier_ids;
  for (const int grid_id : assignment_it->second)
  {
    const auto grid_it = std::find_if(
      grids_.begin(), grids_.end(),
      [grid_id](const GridCellInfo & grid) {
        return grid.id == grid_id;
      });
    if (grid_it == grids_.end())
    {
      continue;
    }
    unique_frontier_ids.insert(
      grid_it->contained_frontier_ids.begin(), grid_it->contained_frontier_ids.end());
  }

  frontier_ids.assign(unique_frontier_ids.begin(), unique_frontier_ids.end());
  return frontier_ids;
}

void HGridAllocator::buildGrid()
{
  grids_.clear();

  const double map_width = static_cast<double>(map_.info.width) * map_.info.resolution;
  const double map_height = static_cast<double>(map_.info.height) * map_.info.resolution;
  const int grid_count_x = std::max(1, static_cast<int>(std::ceil(map_width / params_.grid_size)));
  const int grid_count_y = std::max(1, static_cast<int>(std::ceil(map_height / params_.grid_size)));

  int id = 0;
  for (int gy = 0; gy < grid_count_y; ++gy)
  {
    for (int gx = 0; gx < grid_count_x; ++gx)
    {
      GridCellInfo grid;
      grid.id = id++;
      grid.level = 0;
      grid.size = params_.grid_size;
      grid.center.x() =
        map_.info.origin.position.x + (static_cast<double>(gx) + 0.5) * params_.grid_size;
      grid.center.y() =
        map_.info.origin.position.y + (static_cast<double>(gy) + 0.5) * params_.grid_size;
      grid.center.z() = 0.5;
      grids_.push_back(grid);
    }
  }
}

void HGridAllocator::countGridInformation(const std::vector<FrontierCluster> & frontiers)
{
  for (auto & grid : grids_)
  {
    grid.unknown_num = 0;
    grid.frontier_num = 0;
    grid.free_num = 0;
    grid.active = false;
    grid.contained_frontier_ids.clear();
  }

  for (int my = 0; my < static_cast<int>(map_.info.height); ++my)
  {
    for (int mx = 0; mx < static_cast<int>(map_.info.width); ++mx)
    {
      const double x =
        map_.info.origin.position.x + (static_cast<double>(mx) + 0.5) * map_.info.resolution;
      const double y =
        map_.info.origin.position.y + (static_cast<double>(my) + 0.5) * map_.info.resolution;
      const Eigen::Vector3d point(x, y, 0.0);
      const auto index = static_cast<size_t>(my * static_cast<int>(map_.info.width) + mx);
      if (index >= map_.data.size())
      {
        continue;
      }

      for (auto & grid : grids_)
      {
        if (!pointInGrid(point, grid))
        {
          continue;
        }
        if (map_.data[index] < 0)
        {
          ++grid.unknown_num;
        }
        else if (map_.data[index] == 0)
        {
          ++grid.free_num;
        }
        break;
      }
    }
  }

  for (const auto & frontier : frontiers)
  {
    const auto & cells = frontier.filtered_cells.empty() ? frontier.cells : frontier.filtered_cells;
    for (auto & grid : grids_)
    {
      bool contains_frontier = false;
      int frontier_cells_in_grid = 0;
      for (const auto & cell : cells)
      {
        if (!pointInGrid(cell, grid))
        {
          continue;
        }
        contains_frontier = true;
        ++frontier_cells_in_grid;
      }
      if (contains_frontier)
      {
        grid.contained_frontier_ids.push_back(frontier.id);
        grid.frontier_num += frontier_cells_in_grid;
      }
    }
  }

  for (auto & grid : grids_)
  {
    grid.active = grid.unknown_num >= params_.min_unknown ||
                  grid.frontier_num >= params_.min_frontier ||
                  grid.free_num >= params_.min_free;
  }
}

double HGridAllocator::costDroneToGrid(
  const Eigen::Vector3d & drone_pos,
  const GridCellInfo & grid,
  const int previous_grid_id) const
{
  double consistency_cost = 0.0;
  if (previous_grid_id >= 0)
  {
    consistency_cost =
      previous_grid_id == grid.id ? params_.consistent_cost : params_.consistent_cost2;
  }

  return (drone_pos - grid.center).norm() + consistency_cost -
         params_.w_unknown * static_cast<double>(grid.unknown_num);
}

bool HGridAllocator::pointInGrid(
  const Eigen::Vector3d & point,
  const GridCellInfo & grid) const
{
  const double half_size = 0.5 * grid.size;
  return point.x() >= grid.center.x() - half_size &&
         point.x() < grid.center.x() + half_size &&
         point.y() >= grid.center.y() - half_size &&
         point.y() < grid.center.y() + half_size;
}

}  // namespace exploration_swarm
