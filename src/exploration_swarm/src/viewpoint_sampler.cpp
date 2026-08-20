#include "exploration_swarm/viewpoint_sampler.hpp"

#include <algorithm>
#include <cmath>

namespace exploration_swarm
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

double normalizeAngle(double angle)
{
  while (angle > kPi)
  {
    angle -= kTwoPi;
  }
  while (angle < -kPi)
  {
    angle += kTwoPi;
  }
  return angle;
}
}  // namespace

void ViewpointSampler::setParameters(
  const FrontierParameters & frontier_params,
  const PerceptionParameters & perception_params)
{
  frontier_params_ = frontier_params;
  frontier_params_.candidate_rnum = std::max(1, frontier_params_.candidate_rnum);
  frontier_params_.candidate_dphi = std::max(0.01, frontier_params_.candidate_dphi);
  frontier_params_.candidate_rmin = std::max(0.0, frontier_params_.candidate_rmin);
  frontier_params_.candidate_rmax =
    std::max(frontier_params_.candidate_rmin, frontier_params_.candidate_rmax);
  frontier_params_.min_visib_num = std::max(1, frontier_params_.min_visib_num);
  perception_params_ = perception_params;
  perception_params_.max_dist = std::max(0.1, perception_params_.max_dist);
}

void ViewpointSampler::setMap(const nav_msgs::msg::OccupancyGrid & map)
{
  map_ = map;
  has_map_ = true;
}

std::vector<Viewpoint> ViewpointSampler::sampleViewpoints(const FrontierCluster & frontier)
{
  std::vector<Viewpoint> viewpoints;

  if (!has_map_ || map_.info.width == 0 || map_.info.height == 0 || map_.data.empty())
  {
    return viewpoints;
  }

  const Eigen::Vector3d center = frontier.average;
  const int ring_count = frontier_params_.candidate_rnum;
  const double radius_span = frontier_params_.candidate_rmax - frontier_params_.candidate_rmin;

  for (int ring = 0; ring < ring_count; ++ring)
  {
    const double ratio =
      ring_count == 1 ? 0.0 : static_cast<double>(ring) / static_cast<double>(ring_count - 1);
    const double radius = frontier_params_.candidate_rmin + radius_span * ratio;

    for (double phi = 0.0; phi < kTwoPi; phi += frontier_params_.candidate_dphi)
    {
      Eigen::Vector3d candidate(
        center.x() + radius * std::cos(phi),
        center.y() + radius * std::sin(phi),
        center.z());

      if ((candidate - center).head<2>().norm() < frontier_params_.min_candidate_dist)
      {
        continue;
      }
      if (!isInsideMap(candidate) || !isFree(candidate) || !hasMinClearance(candidate))
      {
        continue;
      }

      const double yaw = std::atan2(center.y() - candidate.y(), center.x() - candidate.x());
      const int visible_num = countVisibleCells(candidate, yaw, frontier);
      if (visible_num < frontier_params_.min_visib_num)
      {
        continue;
      }

      Viewpoint viewpoint;
      viewpoint.position = candidate;
      viewpoint.yaw = yaw;
      viewpoint.visible_num = visible_num;
      viewpoint.frontier_id = frontier.id;
      viewpoints.push_back(viewpoint);
    }
  }

  return viewpoints;
}

bool ViewpointSampler::worldToMap(const Eigen::Vector3d & p, int & mx, int & my) const
{
  if (!has_map_ || map_.info.resolution <= 0.0)
  {
    return false;
  }

  const auto & origin = map_.info.origin;
  const double qx = origin.orientation.x;
  const double qy = origin.orientation.y;
  const double qz = origin.orientation.z;
  const double qw = origin.orientation.w;
  const double yaw = std::atan2(
    2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

  const double dx = p.x() - origin.position.x;
  const double dy = p.y() - origin.position.y;
  const double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
  const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;

  mx = static_cast<int>(std::floor(local_x / map_.info.resolution));
  my = static_cast<int>(std::floor(local_y / map_.info.resolution));

  return mx >= 0 && my >= 0 && mx < static_cast<int>(map_.info.width) &&
         my < static_cast<int>(map_.info.height);
}

Eigen::Vector3d ViewpointSampler::mapToWorld(const int mx, const int my) const
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

  return Eigen::Vector3d(
    origin.position.x + std::cos(yaw) * local_x - std::sin(yaw) * local_y,
    origin.position.y + std::sin(yaw) * local_x + std::cos(yaw) * local_y,
    0.0);
}

bool ViewpointSampler::isInsideMap(const Eigen::Vector3d & p) const
{
  int mx = 0;
  int my = 0;
  return worldToMap(p, mx, my);
}

bool ViewpointSampler::isFree(const Eigen::Vector3d & p) const
{
  int mx = 0;
  int my = 0;
  if (!worldToMap(p, mx, my))
  {
    return false;
  }
  const int index = my * static_cast<int>(map_.info.width) + mx;
  return map_.data[static_cast<size_t>(index)] == 0;
}

bool ViewpointSampler::hasMinClearance(const Eigen::Vector3d & p) const
{
  int center_mx = 0;
  int center_my = 0;
  if (!worldToMap(p, center_mx, center_my))
  {
    return false;
  }

  const double resolution = map_.info.resolution;
  const int radius_cells =
    static_cast<int>(std::ceil(frontier_params_.min_candidate_clearance / resolution));

  for (int dy = -radius_cells; dy <= radius_cells; ++dy)
  {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx)
    {
      const int mx = center_mx + dx;
      const int my = center_my + dy;
      if (mx < 0 || my < 0 || mx >= static_cast<int>(map_.info.width) ||
          my >= static_cast<int>(map_.info.height))
      {
        return false;
      }

      const Eigen::Vector3d cell_center = mapToWorld(mx, my);
      if ((cell_center - p).head<2>().norm() > frontier_params_.min_candidate_clearance)
      {
        continue;
      }

      const int index = my * static_cast<int>(map_.info.width) + mx;
      if (map_.data[static_cast<size_t>(index)] >= 50)
      {
        return false;
      }
    }
  }

  return true;
}

bool ViewpointSampler::hasLineOfSight(
  const Eigen::Vector3d & from, const Eigen::Vector3d & to) const
{
  const Eigen::Vector2d delta = (to - from).head<2>();
  const double distance = delta.norm();
  if (distance <= 1e-6)
  {
    return true;
  }

  const double step = std::max(0.05, static_cast<double>(map_.info.resolution) * 0.5);
  const int step_count = std::max(1, static_cast<int>(std::ceil(distance / step)));

  for (int i = 1; i <= step_count; ++i)
  {
    const double ratio = static_cast<double>(i) / static_cast<double>(step_count);
    const Eigen::Vector3d p = from + (to - from) * ratio;

    int mx = 0;
    int my = 0;
    if (!worldToMap(p, mx, my))
    {
      return false;
    }

    const int index = my * static_cast<int>(map_.info.width) + mx;
    if (map_.data[static_cast<size_t>(index)] >= 50)
    {
      return false;
    }
  }

  return true;
}

int ViewpointSampler::countVisibleCells(
  const Eigen::Vector3d & viewpoint,
  const double yaw,
  const FrontierCluster & frontier) const
{
  int visible_num = 0;
  const auto & cells = frontier.filtered_cells.empty() ? frontier.cells : frontier.filtered_cells;

  for (const auto & cell : cells)
  {
    const Eigen::Vector2d delta = (cell - viewpoint).head<2>();
    const double distance = delta.norm();
    if (distance > perception_params_.max_dist || distance < 1e-6)
    {
      continue;
    }

    const double bearing = std::atan2(delta.y(), delta.x());
    const double yaw_error = normalizeAngle(bearing - yaw);
    if (yaw_error < -perception_params_.right_angle || yaw_error > perception_params_.left_angle)
    {
      continue;
    }

    if (!hasLineOfSight(viewpoint, cell))
    {
      continue;
    }

    ++visible_num;
  }

  return visible_num;
}

}  // namespace exploration_swarm
