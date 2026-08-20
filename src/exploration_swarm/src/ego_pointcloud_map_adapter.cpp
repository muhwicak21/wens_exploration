#include "exploration_swarm/ego_pointcloud_map_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace exploration_swarm
{

EgoPointCloudMapAdapter::EgoPointCloudMapAdapter(
  rclcpp::Node & node, const ExplorationParameters & params)
: node_(node), params_(params)
{
  map_params_.occupied_max_z = params_.map_size_z;

  node_.declare_parameter("map_source.cloud_topic", map_params_.cloud_topic);
  node_.declare_parameter(
    "map_source.local_cloud_topic_suffix", map_params_.local_cloud_topic_suffix);
  node_.declare_parameter(
    "map_source.grid_cloud_topic_suffix", map_params_.grid_cloud_topic_suffix);
  node_.declare_parameter("map_source.occupancy_grid_topic", map_params_.occupancy_grid_topic);
  node_.declare_parameter("map_source.odom_topic_suffix", map_params_.odom_topic_suffix);
  node_.declare_parameter("map_source.resolution", map_params_.resolution);
  node_.declare_parameter("map_source.sensing_radius", map_params_.sensing_radius);
  node_.declare_parameter("map_source.occupied_min_z", map_params_.occupied_min_z);
  node_.declare_parameter("map_source.occupied_max_z", map_params_.occupied_max_z);

  node_.get_parameter("map_source.cloud_topic", map_params_.cloud_topic);
  node_.get_parameter(
    "map_source.local_cloud_topic_suffix", map_params_.local_cloud_topic_suffix);
  node_.get_parameter(
    "map_source.grid_cloud_topic_suffix", map_params_.grid_cloud_topic_suffix);
  node_.get_parameter("map_source.occupancy_grid_topic", map_params_.occupancy_grid_topic);
  node_.get_parameter("map_source.odom_topic_suffix", map_params_.odom_topic_suffix);
  node_.get_parameter("map_source.resolution", map_params_.resolution);
  node_.get_parameter("map_source.sensing_radius", map_params_.sensing_radius);
  node_.get_parameter("map_source.occupied_min_z", map_params_.occupied_min_z);
  node_.get_parameter("map_source.occupied_max_z", map_params_.occupied_max_z);

  map_params_.resolution = std::max(0.05, map_params_.resolution);
  map_params_.sensing_radius = std::max(0.1, map_params_.sensing_radius);
  map_params_.occupied_max_z =
    std::max(map_params_.occupied_min_z, map_params_.occupied_max_z);
  latest_local_clouds_.resize(static_cast<size_t>(params_.drone_num));
  latest_grid_clouds_.resize(static_cast<size_t>(params_.drone_num));
  latest_odoms_.resize(static_cast<size_t>(params_.drone_num));
}

void EgoPointCloudMapAdapter::start()
{
  cloud_sub_ = node_.create_subscription<sensor_msgs::msg::PointCloud2>(
    map_params_.cloud_topic, rclcpp::QoS(1).reliable(),
    std::bind(&EgoPointCloudMapAdapter::onGlobalCloud, this, std::placeholders::_1));

  odom_subs_.reserve(static_cast<size_t>(params_.drone_num));
  local_cloud_subs_.reserve(static_cast<size_t>(params_.drone_num));
  grid_cloud_subs_.reserve(static_cast<size_t>(params_.drone_num));
  for (int drone_id = 0; drone_id < params_.drone_num; ++drone_id)
  {
    std::string odom_suffix = map_params_.odom_topic_suffix;
    if (!odom_suffix.empty() && odom_suffix.front() != '/' && odom_suffix.front() != '_')
    {
      odom_suffix = "_" + odom_suffix;
    }

    const std::string odom_topic =
      "/drone_" + std::to_string(drone_id) + odom_suffix;
    odom_subs_.push_back(node_.create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 20,
      [this, drone_id](const nav_msgs::msg::Odometry::SharedPtr msg) {
        onOdometry(drone_id, msg);
      }));
    RCLCPP_INFO(
      node_.get_logger(), "Subscribed to EGO odometry for map adapter: %s", odom_topic.c_str());

    const std::string local_cloud_topic =
      "/drone_" + std::to_string(drone_id) + "_" + map_params_.local_cloud_topic_suffix;
    local_cloud_subs_.push_back(node_.create_subscription<sensor_msgs::msg::PointCloud2>(
      local_cloud_topic, rclcpp::QoS(5).reliable(),
      [this, drone_id](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        onLocalCloud(drone_id, msg);
      }));
    RCLCPP_INFO(
      node_.get_logger(), "Subscribed to EGO local sensed cloud: %s",
      local_cloud_topic.c_str());

    const std::string grid_cloud_topic =
      "/drone_" + std::to_string(drone_id) + "_" + map_params_.grid_cloud_topic_suffix;
    grid_cloud_subs_.push_back(node_.create_subscription<sensor_msgs::msg::PointCloud2>(
      grid_cloud_topic, rclcpp::QoS(5).reliable(),
      [this, drone_id](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        onGridCloud(drone_id, msg);
      }));
    RCLCPP_INFO(
      node_.get_logger(), "Subscribed to EGO grid occupancy cloud: %s",
      grid_cloud_topic.c_str());
  }

  grid_pub_ = node_.create_publisher<nav_msgs::msg::OccupancyGrid>(
    map_params_.occupancy_grid_topic, rclcpp::QoS(1).reliable().transient_local());

  initializeGrid(node_.now(), params_.frame_id);

  RCLCPP_INFO(
    node_.get_logger(), "Subscribed to EGO global PointCloud2 map for compatibility: %s",
    map_params_.cloud_topic.c_str());
  RCLCPP_INFO(
    node_.get_logger(), "Publishing accumulated local-observation OccupancyGrid: %s",
    map_params_.occupancy_grid_topic.c_str());
}

bool EgoPointCloudMapAdapter::hasMap() const
{
  return has_grid_;
}

const nav_msgs::msg::OccupancyGrid & EgoPointCloudMapAdapter::occupancyGrid() const
{
  return grid_;
}

sensor_msgs::msg::PointCloud2 EgoPointCloudMapAdapter::buildExploredCloud3D(
  const rclcpp::Time & stamp) const
{
  return buildCloudFromVoxelSet(explored_voxels_3d_, stamp);
}

sensor_msgs::msg::PointCloud2 EgoPointCloudMapAdapter::buildOccupiedCloud3D(
  const rclcpp::Time & stamp) const
{
  return buildCloudFromVoxelSet(occupied_voxels_3d_, stamp);
}

void EgoPointCloudMapAdapter::onGlobalCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  latest_global_cloud_ = msg;
}

void EgoPointCloudMapAdapter::onLocalCloud(
  const int drone_id, const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (drone_id < 0 || drone_id >= static_cast<int>(latest_local_clouds_.size()))
  {
    return;
  }

  latest_local_clouds_[static_cast<size_t>(drone_id)] = msg;
  updateGridFromDroneObservation(drone_id, msg->header.stamp);
}

void EgoPointCloudMapAdapter::onGridCloud(
  const int drone_id, const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (drone_id < 0 || drone_id >= static_cast<int>(latest_grid_clouds_.size()))
  {
    return;
  }

  latest_grid_clouds_[static_cast<size_t>(drone_id)] = msg;
  updateGridFromDroneObservation(drone_id, msg->header.stamp);
}

void EgoPointCloudMapAdapter::onOdometry(
  const int drone_id, const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (drone_id < 0 || drone_id >= static_cast<int>(latest_odoms_.size()))
  {
    return;
  }

  latest_odoms_[static_cast<size_t>(drone_id)] = msg;
  updateGridFromDroneObservation(drone_id, msg->header.stamp);
}

void EgoPointCloudMapAdapter::initializeGrid(
  const rclcpp::Time & stamp, const std::string & frame_id)
{
  grid_.header.stamp = stamp;
  grid_.header.frame_id = frame_id.empty() ? params_.frame_id : frame_id;
  grid_.info.resolution = map_params_.resolution;
  grid_.info.width =
    static_cast<uint32_t>(std::ceil(params_.map_size_x / map_params_.resolution));
  grid_.info.height =
    static_cast<uint32_t>(std::ceil(params_.map_size_y / map_params_.resolution));
  grid_.info.origin.position.x = -0.5 * params_.map_size_x;
  grid_.info.origin.position.y = -0.5 * params_.map_size_y;
  grid_.info.origin.position.z = 0.0;
  grid_.info.origin.orientation.w = 1.0;
  grid_.data.assign(static_cast<size_t>(grid_.info.width * grid_.info.height), -1);
  has_grid_ = true;
  grid_pub_->publish(grid_);
}

void EgoPointCloudMapAdapter::updateGridFromDroneObservation(
  const int drone_id, const rclcpp::Time & stamp)
{
  if (!has_grid_)
  {
    initializeGrid(stamp, params_.frame_id);
  }

  grid_.header.stamp = stamp;
  grid_.header.frame_id = params_.frame_id;
  markKnownFreeAroundDrone(drone_id);
  markOccupiedFromLocalCloud(drone_id);
  markOccupiedFromGridCloud(drone_id);
  accumulateLocalCloud3D(drone_id);
  accumulateGridCloud3D(drone_id);
  grid_pub_->publish(grid_);
}

bool EgoPointCloudMapAdapter::worldToMap(
  const double x, const double y, int & mx, int & my) const
{
  mx = static_cast<int>(
    std::floor((x - grid_.info.origin.position.x) / grid_.info.resolution));
  my = static_cast<int>(
    std::floor((y - grid_.info.origin.position.y) / grid_.info.resolution));
  return mx >= 0 && my >= 0 && mx < static_cast<int>(grid_.info.width) &&
         my < static_cast<int>(grid_.info.height);
}

int EgoPointCloudMapAdapter::toIndex(const int mx, const int my) const
{
  return my * static_cast<int>(grid_.info.width) + mx;
}

bool EgoPointCloudMapAdapter::worldToVoxel(
  const double x, const double y, const double z, int & vx, int & vy, int & vz) const
{
  vx = static_cast<int>(
    std::floor((x - grid_.info.origin.position.x) / map_params_.resolution));
  vy = static_cast<int>(
    std::floor((y - grid_.info.origin.position.y) / map_params_.resolution));
  vz = static_cast<int>(std::floor(z / map_params_.resolution));

  return vx >= 0 && vy >= 0 && vz >= 0 &&
         vx < static_cast<int>(grid_.info.width) &&
         vy < static_cast<int>(grid_.info.height) &&
         z <= params_.map_size_z;
}

int64_t EgoPointCloudMapAdapter::voxelKey(const int vx, const int vy, const int vz) const
{
  const int64_t width = static_cast<int64_t>(grid_.info.width);
  const int64_t height = static_cast<int64_t>(grid_.info.height);
  return static_cast<int64_t>(vz) * width * height +
         static_cast<int64_t>(vy) * width +
         static_cast<int64_t>(vx);
}

sensor_msgs::msg::PointCloud2 EgoPointCloudMapAdapter::buildCloudFromVoxelSet(
  const std::unordered_set<int64_t> & voxels,
  const rclcpp::Time & stamp) const
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = params_.frame_id;
  cloud.header.stamp = stamp;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(voxels.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  const int64_t width = static_cast<int64_t>(grid_.info.width);
  const int64_t height = static_cast<int64_t>(grid_.info.height);
  const int64_t layer_size = width * height;

  for (const auto key : voxels)
  {
    const int64_t vz = key / layer_size;
    const int64_t rem = key % layer_size;
    const int64_t vy = rem / width;
    const int64_t vx = rem % width;

    *iter_x = static_cast<float>(
      grid_.info.origin.position.x + (static_cast<double>(vx) + 0.5) * map_params_.resolution);
    *iter_y = static_cast<float>(
      grid_.info.origin.position.y + (static_cast<double>(vy) + 0.5) * map_params_.resolution);
    *iter_z = static_cast<float>((static_cast<double>(vz) + 0.5) * map_params_.resolution);

    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  return cloud;
}

void EgoPointCloudMapAdapter::markKnownFreeAroundDrone(const int drone_id)
{
  if (drone_id < 0 || drone_id >= static_cast<int>(latest_odoms_.size()))
  {
    return;
  }

  const auto & odom = latest_odoms_[static_cast<size_t>(drone_id)];
  if (!odom)
  {
    return;
  }

  const int radius_cells =
    static_cast<int>(std::ceil(map_params_.sensing_radius / grid_.info.resolution));

  int center_mx = 0;
  int center_my = 0;
  if (!worldToMap(
      odom->pose.pose.position.x, odom->pose.pose.position.y, center_mx, center_my))
  {
    return;
  }

  for (int dy = -radius_cells; dy <= radius_cells; ++dy)
  {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx)
    {
      const int mx = center_mx + dx;
      const int my = center_my + dy;
      if (mx < 0 || my < 0 || mx >= static_cast<int>(grid_.info.width) ||
        my >= static_cast<int>(grid_.info.height))
      {
        continue;
      }

      const double dist =
        std::hypot(dx * grid_.info.resolution, dy * grid_.info.resolution);
      if (dist <= map_params_.sensing_radius)
      {
        const auto index = static_cast<size_t>(toIndex(mx, my));
        if (grid_.data[index] < 50)
        {
          grid_.data[index] = 0;
        }
      }
    }
  }
}

void EgoPointCloudMapAdapter::markOccupiedFromLocalCloud(const int drone_id)
{
  if (drone_id < 0 || drone_id >= static_cast<int>(latest_local_clouds_.size()))
  {
    return;
  }

  const auto & cloud = latest_local_clouds_[static_cast<size_t>(drone_id)];
  if (!cloud)
  {
    return;
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    const double x = *iter_x;
    const double y = *iter_y;
    const double z = *iter_z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    {
      continue;
    }
    if (z < map_params_.occupied_min_z || z > map_params_.occupied_max_z)
    {
      continue;
    }

    int mx = 0;
    int my = 0;
    if (!worldToMap(x, y, mx, my))
    {
      continue;
    }

    grid_.data[static_cast<size_t>(toIndex(mx, my))] = 100;
  }
}

void EgoPointCloudMapAdapter::markOccupiedFromGridCloud(const int drone_id)
{
  if (drone_id < 0 || drone_id >= static_cast<int>(latest_grid_clouds_.size()))
  {
    return;
  }

  const auto & cloud = latest_grid_clouds_[static_cast<size_t>(drone_id)];
  if (!cloud)
  {
    return;
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    const double x = *iter_x;
    const double y = *iter_y;
    const double z = *iter_z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    {
      continue;
    }
    if (z < map_params_.occupied_min_z || z > map_params_.occupied_max_z)
    {
      continue;
    }

    int mx = 0;
    int my = 0;
    if (!worldToMap(x, y, mx, my))
    {
      continue;
    }

    grid_.data[static_cast<size_t>(toIndex(mx, my))] = 100;
  }
}

void EgoPointCloudMapAdapter::accumulateLocalCloud3D(const int drone_id)
{
  if (drone_id < 0 || drone_id >= static_cast<int>(latest_local_clouds_.size()))
  {
    return;
  }

  const auto & cloud = latest_local_clouds_[static_cast<size_t>(drone_id)];
  if (!cloud)
  {
    return;
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    const double x = *iter_x;
    const double y = *iter_y;
    const double z = *iter_z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    {
      continue;
    }

    int vx = 0;
    int vy = 0;
    int vz = 0;
    if (!worldToVoxel(x, y, z, vx, vy, vz))
    {
      continue;
    }

    explored_voxels_3d_.insert(voxelKey(vx, vy, vz));
  }
}

void EgoPointCloudMapAdapter::accumulateGridCloud3D(const int drone_id)
{
  if (drone_id < 0 || drone_id >= static_cast<int>(latest_grid_clouds_.size()))
  {
    return;
  }

  const auto & cloud = latest_grid_clouds_[static_cast<size_t>(drone_id)];
  if (!cloud)
  {
    return;
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    const double x = *iter_x;
    const double y = *iter_y;
    const double z = *iter_z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    {
      continue;
    }
    if (z < map_params_.occupied_min_z || z > map_params_.occupied_max_z)
    {
      continue;
    }

    int vx = 0;
    int vy = 0;
    int vz = 0;
    if (!worldToVoxel(x, y, z, vx, vy, vz))
    {
      continue;
    }

    const auto key = voxelKey(vx, vy, vz);
    explored_voxels_3d_.insert(key);
    occupied_voxels_3d_.insert(key);
  }
}

}  // namespace exploration_swarm
