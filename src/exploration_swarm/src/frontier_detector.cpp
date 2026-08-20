#include "exploration_swarm/frontier_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <queue>

namespace exploration_swarm
{

void FrontierDetector::setParameters(const FrontierParameters & params, const double default_goal_z)
{
  params_ = params;
  params_.cluster_min = std::max(1, params_.cluster_min);
  params_.down_sample = std::max(1, params_.down_sample);
  default_goal_z_ = default_goal_z;
}

void FrontierDetector::setMap(const nav_msgs::msg::OccupancyGrid & map)
{
  map_ = map;
  has_map_ = true;
}

std::vector<FrontierCluster> FrontierDetector::detectFrontiers()
{
  std::vector<FrontierCluster> clusters;

  if (!has_map_ || map_.info.width == 0 || map_.info.height == 0 || map_.data.empty())
  {
    return clusters;
  }

  visited_.assign(map_.data.size(), false);

  int next_cluster_id = 0;
  for (int my = 0; my < static_cast<int>(map_.info.height); ++my)
  {
    for (int mx = 0; mx < static_cast<int>(map_.info.width); ++mx)
    {
      const int index = toIndex(mx, my);
      if (visited_[static_cast<size_t>(index)] || !isFrontierCell(mx, my))
      {
        continue;
      }

      FrontierCluster cluster;
      cluster.id = next_cluster_id;
      cluster.label = FrontierLabel::FRONTIER;
      expandFrontierBFS(mx, my, cluster);

      if (static_cast<int>(cluster.cells.size()) < params_.cluster_min)
      {
        continue;
      }

      computeFrontierInfo(cluster);
      downsampleFrontier(cluster);
      clusters.push_back(cluster);
      ++next_cluster_id;
    }
  }

  return clusters;
}

bool FrontierDetector::isInside(const int mx, const int my) const
{
  return mx >= 0 && my >= 0 && mx < static_cast<int>(map_.info.width) &&
         my < static_cast<int>(map_.info.height);
}

int FrontierDetector::toIndex(const int mx, const int my) const
{
  return my * static_cast<int>(map_.info.width) + mx;
}

Eigen::Vector3d FrontierDetector::mapToWorld(const int mx, const int my) const
{
  const double resolution = map_.info.resolution;
  const double local_x = (static_cast<double>(mx) + 0.5) * resolution;
  const double local_y = (static_cast<double>(my) + 0.5) * resolution;

  const auto & origin = map_.info.origin;
  const double qx = origin.orientation.x;
  const double qy = origin.orientation.y;
  const double qz = origin.orientation.z;
  const double qw = origin.orientation.w;
  const double yaw = std::atan2(
    2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  return Eigen::Vector3d(
    origin.position.x + cos_yaw * local_x - sin_yaw * local_y,
    origin.position.y + sin_yaw * local_x + cos_yaw * local_y,
    default_goal_z_);
}

bool FrontierDetector::isKnownFree(const int mx, const int my) const
{
  if (!isInside(mx, my))
  {
    return false;
  }
  return map_.data[static_cast<size_t>(toIndex(mx, my))] == 0;
}

bool FrontierDetector::isUnknown(const int mx, const int my) const
{
  if (!isInside(mx, my))
  {
    return false;
  }
  return map_.data[static_cast<size_t>(toIndex(mx, my))] == -1;
}

bool FrontierDetector::isOccupied(const int mx, const int my) const
{
  if (!isInside(mx, my))
  {
    return true;
  }
  return map_.data[static_cast<size_t>(toIndex(mx, my))] >= 50;
}

bool FrontierDetector::isFrontierCell(const int mx, const int my) const
{
  if (!isKnownFree(mx, my) || isOccupied(mx, my))
  {
    return false;
  }

  for (int dy = -1; dy <= 1; ++dy)
  {
    for (int dx = -1; dx <= 1; ++dx)
    {
      if (dx == 0 && dy == 0)
      {
        continue;
      }
      if (isUnknown(mx + dx, my + dy))
      {
        return true;
      }
    }
  }

  return false;
}

void FrontierDetector::expandFrontierBFS(
  const int start_x, const int start_y, FrontierCluster & cluster)
{
  std::queue<std::pair<int, int>> frontier_queue;
  frontier_queue.emplace(start_x, start_y);
  visited_[static_cast<size_t>(toIndex(start_x, start_y))] = true;

  while (!frontier_queue.empty())
  {
    const auto [mx, my] = frontier_queue.front();
    frontier_queue.pop();

    if (!isFrontierCell(mx, my))
    {
      continue;
    }

    cluster.cells.push_back(mapToWorld(mx, my));

    for (int dy = -1; dy <= 1; ++dy)
    {
      for (int dx = -1; dx <= 1; ++dx)
      {
        if (dx == 0 && dy == 0)
        {
          continue;
        }

        const int nx = mx + dx;
        const int ny = my + dy;
        if (!isInside(nx, ny))
        {
          continue;
        }

        const int neighbor_index = toIndex(nx, ny);
        if (visited_[static_cast<size_t>(neighbor_index)])
        {
          continue;
        }

        visited_[static_cast<size_t>(neighbor_index)] = true;
        if (isFrontierCell(nx, ny))
        {
          frontier_queue.emplace(nx, ny);
        }
      }
    }
  }
}

void FrontierDetector::computeFrontierInfo(FrontierCluster & cluster)
{
  if (cluster.cells.empty())
  {
    cluster.average = Eigen::Vector3d::Zero();
    cluster.box_min = Eigen::Vector3d::Zero();
    cluster.box_max = Eigen::Vector3d::Zero();
    return;
  }

  cluster.average = Eigen::Vector3d::Zero();
  cluster.box_min = Eigen::Vector3d(
    std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
    std::numeric_limits<double>::max());
  cluster.box_max = Eigen::Vector3d(
    std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(),
    std::numeric_limits<double>::lowest());

  for (const auto & cell : cluster.cells)
  {
    cluster.average += cell;
    cluster.box_min = cluster.box_min.cwiseMin(cell);
    cluster.box_max = cluster.box_max.cwiseMax(cell);
  }

  cluster.average /= static_cast<double>(cluster.cells.size());
}

void FrontierDetector::downsampleFrontier(FrontierCluster & cluster)
{
  cluster.filtered_cells.clear();
  cluster.filtered_cells.reserve(
    (cluster.cells.size() + static_cast<size_t>(params_.down_sample) - 1) /
    static_cast<size_t>(params_.down_sample));

  for (size_t i = 0; i < cluster.cells.size(); i += static_cast<size_t>(params_.down_sample))
  {
    cluster.filtered_cells.push_back(cluster.cells[i]);
  }
}

}  // namespace exploration_swarm
