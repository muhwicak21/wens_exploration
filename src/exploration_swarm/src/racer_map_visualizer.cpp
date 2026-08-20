#include "exploration_swarm/racer_map_visualizer.hpp"

#include <algorithm>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace exploration_swarm
{

void RacerMapVisualizer::setParameters(
  const RacerVisualizerParameters & params,
  const std::string & frame_id)
{
  params_ = params;
  params_.occupied_threshold = std::max(1, params_.occupied_threshold);
  params_.unknown_downsample = std::max(1, params_.unknown_downsample);
  params_.explored_downsample = std::max(1, params_.explored_downsample);
  params_.occupied_downsample = std::max(1, params_.occupied_downsample);
  frame_id_ = frame_id;
}

void RacerMapVisualizer::setStamp(const rclcpp::Time & stamp)
{
  stamp_ = stamp;
}

void RacerMapVisualizer::setMap(const nav_msgs::msg::OccupancyGrid & map)
{
  map_ = map;
  has_map_ = true;
}

sensor_msgs::msg::PointCloud2 RacerMapVisualizer::buildExploredCloud() const
{
  std::vector<Eigen::Vector3d> points;
  if (!has_map_)
  {
    return buildCloudFromCells(points);
  }

  for (int my = 0; my < static_cast<int>(map_.info.height); my += params_.explored_downsample)
  {
    for (int mx = 0; mx < static_cast<int>(map_.info.width); mx += params_.explored_downsample)
    {
      const int index = my * static_cast<int>(map_.info.width) + mx;
      if (isKnown(map_.data[static_cast<size_t>(index)]))
      {
        points.push_back(gridToWorld(mx, my, params_.visualization_z));
      }
    }
  }

  return buildCloudFromCells(points);
}

sensor_msgs::msg::PointCloud2 RacerMapVisualizer::buildOccupiedCloud() const
{
  std::vector<Eigen::Vector3d> points;
  if (!has_map_)
  {
    return buildCloudFromCells(points);
  }

  for (int my = 0; my < static_cast<int>(map_.info.height); my += params_.occupied_downsample)
  {
    for (int mx = 0; mx < static_cast<int>(map_.info.width); mx += params_.occupied_downsample)
    {
      const int index = my * static_cast<int>(map_.info.width) + mx;
      if (isOccupied(map_.data[static_cast<size_t>(index)]))
      {
        points.push_back(gridToWorld(mx, my, params_.visualization_z));
      }
    }
  }

  return buildCloudFromCells(points);
}

sensor_msgs::msg::PointCloud2 RacerMapVisualizer::buildUnknownCloud() const
{
  std::vector<Eigen::Vector3d> points;
  if (!has_map_)
  {
    return buildCloudFromCells(points);
  }

  for (int my = 0; my < static_cast<int>(map_.info.height); my += params_.unknown_downsample)
  {
    for (int mx = 0; mx < static_cast<int>(map_.info.width); mx += params_.unknown_downsample)
    {
      const int index = my * static_cast<int>(map_.info.width) + mx;
      if (isUnknown(map_.data[static_cast<size_t>(index)]))
      {
        points.push_back(gridToWorld(mx, my, params_.visualization_z));
      }
    }
  }

  return buildCloudFromCells(points);
}

bool RacerMapVisualizer::isKnown(const int value) const
{
  return value != params_.unknown_value;
}

bool RacerMapVisualizer::isFree(const int value) const
{
  return value == params_.free_value;
}

bool RacerMapVisualizer::isOccupied(const int value) const
{
  return value >= params_.occupied_threshold;
}

bool RacerMapVisualizer::isUnknown(const int value) const
{
  return value == params_.unknown_value;
}

Eigen::Vector3d RacerMapVisualizer::gridToWorld(
  const int mx, const int my, const double z) const
{
  return Eigen::Vector3d(
    map_.info.origin.position.x + (static_cast<double>(mx) + 0.5) * map_.info.resolution,
    map_.info.origin.position.y + (static_cast<double>(my) + 0.5) * map_.info.resolution,
    z);
}

sensor_msgs::msg::PointCloud2 RacerMapVisualizer::buildCloudFromCells(
  const std::vector<Eigen::Vector3d> & points) const
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id_;
  cloud.header.stamp = stamp_;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (const auto & point : points)
  {
    *iter_x = static_cast<float>(point.x());
    *iter_y = static_cast<float>(point.y());
    *iter_z = static_cast<float>(point.z());
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  return cloud;
}

}  // namespace exploration_swarm
