#include "exploration_swarm/exploration_goal_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>

namespace exploration_swarm
{

ExplorationGoalManager::ExplorationGoalManager(rclcpp::Node & node)
: node_(node), visualizer_(node)
{
  declareParameters();
  readParameters();
  current_goals_.resize(static_cast<size_t>(params_.drone_num));
  previous_goals_.resize(static_cast<size_t>(params_.drone_num));
  has_current_goals_.assign(static_cast<size_t>(params_.drone_num), false);
  has_previous_goals_.assign(static_cast<size_t>(params_.drone_num), false);
  current_goal_stamps_.resize(static_cast<size_t>(params_.drone_num));
  previous_grid_ids_.assign(static_cast<size_t>(params_.drone_num), -1);
  current_roles_.assign(static_cast<size_t>(params_.drone_num), Role::UNKNOWN);
  frontier_detector_.setParameters(params_.frontier, params_.default_goal_z);
  viewpoint_sampler_.setParameters(params_.frontier, params_.perception);
  hgrid_allocator_.setParameters(params_.partitioning);
  role_assigner_.setParameters(params_.role_assigner);
  visualizer_.setFrameId(params_.frame_id);
  visualizer_.setPerceptionParameters(params_.perception);
  racer_map_visualizer_.setParameters(params_.racer_visualizer, params_.frame_id);
  createGoalPublishers();
  createOdometrySubscriptions();

  if (params_.racer_visualizer.enable_racer_visualizer)
  {
    explored_area_pub_ =
      node_.create_publisher<sensor_msgs::msg::PointCloud2>(
      "/swarm_exploration/explored_area", 1);
    occupied_area_pub_ =
      node_.create_publisher<sensor_msgs::msg::PointCloud2>(
      "/swarm_exploration/occupied_area", 1);
    unknown_area_pub_ =
      node_.create_publisher<sensor_msgs::msg::PointCloud2>(
      "/swarm_exploration/unknown_area", 1);
    explored_area_3d_pub_ =
      node_.create_publisher<sensor_msgs::msg::PointCloud2>(
      "/swarm_exploration/explored_area_3d", 1);
    occupied_area_3d_pub_ =
      node_.create_publisher<sensor_msgs::msg::PointCloud2>(
      "/swarm_exploration/occupied_area_3d", 1);
  }

  if (params_.map_source_type == "pointcloud")
  {
    ego_map_adapter_ = std::make_unique<EgoPointCloudMapAdapter>(node_, params_);
    ego_map_adapter_->start();
  }

  map_sub_ = node_.create_subscription<nav_msgs::msg::OccupancyGrid>(
    params_.map_topic, rclcpp::QoS(1).reliable(),
    std::bind(&ExplorationGoalManager::onMap, this, std::placeholders::_1));

  timer_ = node_.create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(params_.publish_period)),
    std::bind(&ExplorationGoalManager::onTimer, this));

  RCLCPP_INFO(
    node_.get_logger(), "Waiting for map and odometry for frontier-based exploration.");
  RCLCPP_INFO(node_.get_logger(), "Map source type: %s", params_.map_source_type.c_str());
  RCLCPP_INFO(
    node_.get_logger(), "Subscribed to OccupancyGrid topic: %s", params_.map_topic.c_str());
  RCLCPP_INFO(
    node_.get_logger(), "RACER-style visualizer: %s",
    params_.racer_visualizer.enable_racer_visualizer ? "enabled" : "disabled");
  RCLCPP_INFO(
    node_.get_logger(),
    "Exploration feature configuration:\n"
    "  multi-drone separation: %s\n"
    "  HGrid: %s\n"
    "  role assignment: %s",
    params_.enable_multi_drone_separation ? "enabled" : "disabled",
    params_.enable_hgrid ? "enabled" : "disabled",
    params_.enable_role_assignment ? "enabled" : "disabled");
}

const ExplorationParameters & ExplorationGoalManager::parameters() const
{
  return params_;
}

void ExplorationGoalManager::declareParameters()
{
  node_.declare_parameter("drone_num", params_.drone_num);
  node_.declare_parameter("map_source_type", params_.map_source_type);
  node_.declare_parameter("map_topic", params_.map_topic);
  node_.declare_parameter("odom_topic_prefix", params_.odom_topic_prefix);
  node_.declare_parameter("odom_topic_suffix", params_.odom_topic_suffix);
  node_.declare_parameter("frame_id", params_.frame_id);
  node_.declare_parameter("map_size_x", params_.map_size_x);
  node_.declare_parameter("map_size_y", params_.map_size_y);
  node_.declare_parameter("map_size_z", params_.map_size_z);
  node_.declare_parameter("map_margin", params_.map_margin);
  node_.declare_parameter("min_z", params_.min_z);
  node_.declare_parameter("default_goal_z", params_.default_goal_z);
  node_.declare_parameter("publish_period", params_.publish_period);
  node_.declare_parameter("publish_only_when_reached", params_.publish_only_when_reached);
  node_.declare_parameter("goal_reached_threshold", params_.goal_reached_threshold);
  node_.declare_parameter("goal_timeout", params_.goal_timeout);
  node_.declare_parameter("min_goal_distance", params_.min_goal_distance);
  node_.declare_parameter("max_goal_distance", params_.max_goal_distance);
  node_.declare_parameter("min_goal_separation", params_.min_goal_separation);
  node_.declare_parameter(
    "enable_multi_drone_separation", params_.enable_multi_drone_separation);
  node_.declare_parameter("enable_hgrid", params_.enable_hgrid);
  node_.declare_parameter("enable_role_assignment", params_.enable_role_assignment);
  node_.declare_parameter("communication_range", params_.collaboration.communication_range);
  node_.declare_parameter("active_collaboration", params_.collaboration.active_collaboration);
  node_.declare_parameter("frontier.cluster_min", params_.frontier.cluster_min);
  node_.declare_parameter("frontier.cluster_size_xy", params_.frontier.cluster_size_xy);
  node_.declare_parameter("frontier.cluster_size_z", params_.frontier.cluster_size_z);
  node_.declare_parameter("frontier.min_candidate_dist", params_.frontier.min_candidate_dist);
  node_.declare_parameter(
    "frontier.min_candidate_clearance", params_.frontier.min_candidate_clearance);
  node_.declare_parameter("frontier.candidate_dphi", params_.frontier.candidate_dphi);
  node_.declare_parameter("frontier.candidate_rnum", params_.frontier.candidate_rnum);
  node_.declare_parameter("frontier.candidate_rmin", params_.frontier.candidate_rmin);
  node_.declare_parameter("frontier.candidate_rmax", params_.frontier.candidate_rmax);
  node_.declare_parameter("frontier.down_sample", params_.frontier.down_sample);
  node_.declare_parameter("frontier.min_visib_num", params_.frontier.min_visib_num);
  node_.declare_parameter(
    "frontier.min_view_finish_fraction", params_.frontier.min_view_finish_fraction);
  node_.declare_parameter("perception_utils.top_angle", params_.perception.top_angle);
  node_.declare_parameter("perception_utils.left_angle", params_.perception.left_angle);
  node_.declare_parameter("perception_utils.right_angle", params_.perception.right_angle);
  node_.declare_parameter("perception_utils.max_dist", params_.perception.max_dist);
  node_.declare_parameter("perception_utils.vis_dist", params_.perception.vis_dist);
  node_.declare_parameter("exploration.refine_local", params_.exploration.refine_local);
  node_.declare_parameter("exploration.refined_num", params_.exploration.refined_num);
  node_.declare_parameter("exploration.refined_radius", params_.exploration.refined_radius);
  node_.declare_parameter("exploration.max_decay", params_.exploration.max_decay);
  node_.declare_parameter("exploration.top_view_num", params_.exploration.top_view_num);
  node_.declare_parameter("exploration.vm", params_.exploration.vm);
  node_.declare_parameter("exploration.am", params_.exploration.am);
  node_.declare_parameter("exploration.yd", params_.exploration.yd);
  node_.declare_parameter("exploration.ydd", params_.exploration.ydd);
  node_.declare_parameter("exploration.w_dir", params_.exploration.w_dir);
  node_.declare_parameter("exploration.init_plan_num", params_.exploration.init_plan_num);
  node_.declare_parameter("explorer.ftr_max_distance", params_.explorer.ftr_max_distance);
  node_.declare_parameter("explorer.max_ang_dist", params_.explorer.max_ang_dist);
  node_.declare_parameter("explorer.label_penalty", params_.explorer.label_penalty);
  node_.declare_parameter("explorer.w_distance", params_.explorer.w_distance);
  node_.declare_parameter("explorer.w_direction", params_.explorer.w_direction);
  node_.declare_parameter("explorer.w_others", params_.explorer.w_others);
  node_.declare_parameter("explorer.w_previous_goal", params_.explorer.w_previous_goal);
  node_.declare_parameter("collab_assigner.dist_range", params_.collaboration.dist_range);
  node_.declare_parameter("collab_assigner.dist_collision", params_.collaboration.dist_collision);
  node_.declare_parameter("collab_assigner.w_range", params_.collaboration.w_range);
  node_.declare_parameter("collab_assigner.w_collision", params_.collaboration.w_collision);
  node_.declare_parameter("collab_assigner.active", params_.collaboration.active);
  node_.declare_parameter("partitioning.min_unknown", params_.partitioning.min_unknown);
  node_.declare_parameter("partitioning.min_frontier", params_.partitioning.min_frontier);
  node_.declare_parameter("partitioning.min_free", params_.partitioning.min_free);
  node_.declare_parameter("partitioning.consistent_cost", params_.partitioning.consistent_cost);
  node_.declare_parameter("partitioning.consistent_cost2", params_.partitioning.consistent_cost2);
  node_.declare_parameter("partitioning.w_unknown", params_.partitioning.w_unknown);
  node_.declare_parameter("partitioning.grid_size", params_.partitioning.grid_size);
  node_.declare_parameter("partitioning.use_swarm_tf", params_.partitioning.use_swarm_tf);
  node_.declare_parameter("role_assigner.region_size", params_.role_assigner.region_size);
  node_.declare_parameter("role_assigner.cluster_xy_size", params_.role_assigner.cluster_xy_size);
  node_.declare_parameter("role_assigner.min_num_neighbours", params_.role_assigner.min_num_neighbours);
  node_.declare_parameter("role_assigner.min_num_islands", params_.role_assigner.min_num_islands);
  node_.declare_parameter("role_assigner.min_num_trails", params_.role_assigner.min_num_trails);
  node_.declare_parameter("role_assigner.fixed", params_.role_assigner.fixed);
  node_.declare_parameter("role_assigner.fix_role", params_.role_assigner.fix_role);
  node_.declare_parameter("collector.min_dist_collision", params_.collector.min_dist_collision);
  node_.declare_parameter("collector.min_vel", params_.collector.min_vel);
  node_.declare_parameter("collector.label_penalty", params_.collector.label_penalty);
  node_.declare_parameter("collector.velocity_factor", params_.collector.velocity_factor);
  node_.declare_parameter("collector.w_distance", params_.collector.w_distance);
  node_.declare_parameter("collector.w_direction", params_.collector.w_direction);
  node_.declare_parameter("collector.w_others", params_.collector.w_others);
  node_.declare_parameter("collector.w_previous_goal", params_.collector.w_previous_goal);
  node_.declare_parameter("potential_field.ka", params_.potential_field.ka);
  node_.declare_parameter("potential_field.kr", params_.potential_field.kr);
  node_.declare_parameter("potential_field.d0", params_.potential_field.d0);
  node_.declare_parameter("potential_field.df", params_.potential_field.df);
  node_.declare_parameter("potential_field.dc", params_.potential_field.dc);
  node_.declare_parameter("occupied_threshold", params_.racer_visualizer.occupied_threshold);
  node_.declare_parameter("free_value", params_.racer_visualizer.free_value);
  node_.declare_parameter("unknown_value", params_.racer_visualizer.unknown_value);
  node_.declare_parameter(
    "enable_racer_visualizer", params_.racer_visualizer.enable_racer_visualizer);
  node_.declare_parameter(
    "publish_explored_cloud", params_.racer_visualizer.publish_explored_cloud);
  node_.declare_parameter(
    "publish_occupied_cloud", params_.racer_visualizer.publish_occupied_cloud);
  node_.declare_parameter(
    "publish_unknown_cloud", params_.racer_visualizer.publish_unknown_cloud);
  node_.declare_parameter("visualization_period", params_.racer_visualizer.visualization_period);
  node_.declare_parameter("visualization_z", params_.racer_visualizer.visualization_z);
  node_.declare_parameter("unknown_downsample", params_.racer_visualizer.unknown_downsample);
  node_.declare_parameter("explored_downsample", params_.racer_visualizer.explored_downsample);
  node_.declare_parameter("occupied_downsample", params_.racer_visualizer.occupied_downsample);
}

void ExplorationGoalManager::readParameters()
{
  node_.get_parameter("drone_num", params_.drone_num);
  node_.get_parameter("map_source_type", params_.map_source_type);
  node_.get_parameter("map_topic", params_.map_topic);
  node_.get_parameter("odom_topic_prefix", params_.odom_topic_prefix);
  node_.get_parameter("odom_topic_suffix", params_.odom_topic_suffix);
  node_.get_parameter("frame_id", params_.frame_id);
  node_.get_parameter("map_size_x", params_.map_size_x);
  node_.get_parameter("map_size_y", params_.map_size_y);
  node_.get_parameter("map_size_z", params_.map_size_z);
  node_.get_parameter("map_margin", params_.map_margin);
  node_.get_parameter("min_z", params_.min_z);
  node_.get_parameter("default_goal_z", params_.default_goal_z);
  node_.get_parameter("publish_period", params_.publish_period);
  node_.get_parameter("publish_only_when_reached", params_.publish_only_when_reached);
  node_.get_parameter("goal_reached_threshold", params_.goal_reached_threshold);
  node_.get_parameter("goal_timeout", params_.goal_timeout);
  node_.get_parameter("min_goal_distance", params_.min_goal_distance);
  node_.get_parameter("max_goal_distance", params_.max_goal_distance);
  node_.get_parameter("min_goal_separation", params_.min_goal_separation);
  node_.get_parameter(
    "enable_multi_drone_separation", params_.enable_multi_drone_separation);
  node_.get_parameter("enable_hgrid", params_.enable_hgrid);
  node_.get_parameter("enable_role_assignment", params_.enable_role_assignment);
  node_.get_parameter("communication_range", params_.collaboration.communication_range);
  node_.get_parameter("active_collaboration", params_.collaboration.active_collaboration);
  node_.get_parameter("frontier.cluster_min", params_.frontier.cluster_min);
  node_.get_parameter("frontier.cluster_size_xy", params_.frontier.cluster_size_xy);
  node_.get_parameter("frontier.cluster_size_z", params_.frontier.cluster_size_z);
  node_.get_parameter("frontier.min_candidate_dist", params_.frontier.min_candidate_dist);
  node_.get_parameter(
    "frontier.min_candidate_clearance", params_.frontier.min_candidate_clearance);
  node_.get_parameter("frontier.candidate_dphi", params_.frontier.candidate_dphi);
  node_.get_parameter("frontier.candidate_rnum", params_.frontier.candidate_rnum);
  node_.get_parameter("frontier.candidate_rmin", params_.frontier.candidate_rmin);
  node_.get_parameter("frontier.candidate_rmax", params_.frontier.candidate_rmax);
  node_.get_parameter("frontier.down_sample", params_.frontier.down_sample);
  node_.get_parameter("frontier.min_visib_num", params_.frontier.min_visib_num);
  node_.get_parameter(
    "frontier.min_view_finish_fraction", params_.frontier.min_view_finish_fraction);
  node_.get_parameter("perception_utils.top_angle", params_.perception.top_angle);
  node_.get_parameter("perception_utils.left_angle", params_.perception.left_angle);
  node_.get_parameter("perception_utils.right_angle", params_.perception.right_angle);
  node_.get_parameter("perception_utils.max_dist", params_.perception.max_dist);
  node_.get_parameter("perception_utils.vis_dist", params_.perception.vis_dist);
  node_.get_parameter("exploration.refine_local", params_.exploration.refine_local);
  node_.get_parameter("exploration.refined_num", params_.exploration.refined_num);
  node_.get_parameter("exploration.refined_radius", params_.exploration.refined_radius);
  node_.get_parameter("exploration.max_decay", params_.exploration.max_decay);
  node_.get_parameter("exploration.top_view_num", params_.exploration.top_view_num);
  node_.get_parameter("exploration.vm", params_.exploration.vm);
  node_.get_parameter("exploration.am", params_.exploration.am);
  node_.get_parameter("exploration.yd", params_.exploration.yd);
  node_.get_parameter("exploration.ydd", params_.exploration.ydd);
  node_.get_parameter("exploration.w_dir", params_.exploration.w_dir);
  node_.get_parameter("exploration.init_plan_num", params_.exploration.init_plan_num);
  node_.get_parameter("explorer.ftr_max_distance", params_.explorer.ftr_max_distance);
  node_.get_parameter("explorer.max_ang_dist", params_.explorer.max_ang_dist);
  node_.get_parameter("explorer.label_penalty", params_.explorer.label_penalty);
  node_.get_parameter("explorer.w_distance", params_.explorer.w_distance);
  node_.get_parameter("explorer.w_direction", params_.explorer.w_direction);
  node_.get_parameter("explorer.w_others", params_.explorer.w_others);
  node_.get_parameter("explorer.w_previous_goal", params_.explorer.w_previous_goal);
  node_.get_parameter("collab_assigner.dist_range", params_.collaboration.dist_range);
  node_.get_parameter("collab_assigner.dist_collision", params_.collaboration.dist_collision);
  node_.get_parameter("collab_assigner.w_range", params_.collaboration.w_range);
  node_.get_parameter("collab_assigner.w_collision", params_.collaboration.w_collision);
  node_.get_parameter("collab_assigner.active", params_.collaboration.active);
  node_.get_parameter("partitioning.min_unknown", params_.partitioning.min_unknown);
  node_.get_parameter("partitioning.min_frontier", params_.partitioning.min_frontier);
  node_.get_parameter("partitioning.min_free", params_.partitioning.min_free);
  node_.get_parameter("partitioning.consistent_cost", params_.partitioning.consistent_cost);
  node_.get_parameter("partitioning.consistent_cost2", params_.partitioning.consistent_cost2);
  node_.get_parameter("partitioning.w_unknown", params_.partitioning.w_unknown);
  node_.get_parameter("partitioning.grid_size", params_.partitioning.grid_size);
  node_.get_parameter("partitioning.use_swarm_tf", params_.partitioning.use_swarm_tf);
  node_.get_parameter("role_assigner.region_size", params_.role_assigner.region_size);
  node_.get_parameter("role_assigner.cluster_xy_size", params_.role_assigner.cluster_xy_size);
  node_.get_parameter("role_assigner.min_num_neighbours", params_.role_assigner.min_num_neighbours);
  node_.get_parameter("role_assigner.min_num_islands", params_.role_assigner.min_num_islands);
  node_.get_parameter("role_assigner.min_num_trails", params_.role_assigner.min_num_trails);
  node_.get_parameter("role_assigner.fixed", params_.role_assigner.fixed);
  node_.get_parameter("role_assigner.fix_role", params_.role_assigner.fix_role);
  node_.get_parameter("collector.min_dist_collision", params_.collector.min_dist_collision);
  node_.get_parameter("collector.min_vel", params_.collector.min_vel);
  node_.get_parameter("collector.label_penalty", params_.collector.label_penalty);
  node_.get_parameter("collector.velocity_factor", params_.collector.velocity_factor);
  node_.get_parameter("collector.w_distance", params_.collector.w_distance);
  node_.get_parameter("collector.w_direction", params_.collector.w_direction);
  node_.get_parameter("collector.w_others", params_.collector.w_others);
  node_.get_parameter("collector.w_previous_goal", params_.collector.w_previous_goal);
  node_.get_parameter("potential_field.ka", params_.potential_field.ka);
  node_.get_parameter("potential_field.kr", params_.potential_field.kr);
  node_.get_parameter("potential_field.d0", params_.potential_field.d0);
  node_.get_parameter("potential_field.df", params_.potential_field.df);
  node_.get_parameter("potential_field.dc", params_.potential_field.dc);
  node_.get_parameter("occupied_threshold", params_.racer_visualizer.occupied_threshold);
  node_.get_parameter("free_value", params_.racer_visualizer.free_value);
  node_.get_parameter("unknown_value", params_.racer_visualizer.unknown_value);
  node_.get_parameter(
    "enable_racer_visualizer", params_.racer_visualizer.enable_racer_visualizer);
  node_.get_parameter(
    "publish_explored_cloud", params_.racer_visualizer.publish_explored_cloud);
  node_.get_parameter(
    "publish_occupied_cloud", params_.racer_visualizer.publish_occupied_cloud);
  node_.get_parameter("publish_unknown_cloud", params_.racer_visualizer.publish_unknown_cloud);
  node_.get_parameter("visualization_period", params_.racer_visualizer.visualization_period);
  node_.get_parameter("visualization_z", params_.racer_visualizer.visualization_z);
  node_.get_parameter("unknown_downsample", params_.racer_visualizer.unknown_downsample);
  node_.get_parameter("explored_downsample", params_.racer_visualizer.explored_downsample);
  node_.get_parameter("occupied_downsample", params_.racer_visualizer.occupied_downsample);

  params_.drone_num = std::max(1, params_.drone_num);
  params_.publish_period = std::max(0.1, params_.publish_period);
  params_.frontier.cluster_min = std::max(1, params_.frontier.cluster_min);
  params_.frontier.down_sample = std::max(1, params_.frontier.down_sample);
  params_.frontier.candidate_rnum = std::max(1, params_.frontier.candidate_rnum);
  params_.frontier.candidate_dphi = std::max(0.01, params_.frontier.candidate_dphi);
  params_.frontier.candidate_rmin = std::max(0.0, params_.frontier.candidate_rmin);
  params_.frontier.candidate_rmax =
    std::max(params_.frontier.candidate_rmin, params_.frontier.candidate_rmax);
  params_.frontier.min_visib_num = std::max(1, params_.frontier.min_visib_num);
  params_.perception.max_dist = std::max(0.1, params_.perception.max_dist);
  params_.exploration.refined_num = std::max(1, params_.exploration.refined_num);
  params_.exploration.top_view_num = std::max(1, params_.exploration.top_view_num);
  params_.explorer.ftr_max_distance = std::max(0.1, params_.explorer.ftr_max_distance);
  params_.explorer.max_ang_dist = std::max(0.01, params_.explorer.max_ang_dist);
  params_.collaboration.communication_range =
    std::max(0.1, params_.collaboration.communication_range);
  params_.collaboration.dist_range = std::max(0.1, params_.collaboration.dist_range);
  params_.collaboration.dist_collision = std::max(0.1, params_.collaboration.dist_collision);
  params_.partitioning.min_unknown = std::max(0, params_.partitioning.min_unknown);
  params_.partitioning.min_frontier = std::max(0, params_.partitioning.min_frontier);
  params_.partitioning.min_free = std::max(0, params_.partitioning.min_free);
  params_.partitioning.grid_size = std::max(1.0, params_.partitioning.grid_size);
  params_.role_assigner.region_size = std::max(0.1, params_.role_assigner.region_size);
  params_.role_assigner.cluster_xy_size = std::max(0.1, params_.role_assigner.cluster_xy_size);
  params_.role_assigner.min_num_neighbours = std::max(0, params_.role_assigner.min_num_neighbours);
  params_.role_assigner.min_num_islands = std::max(0, params_.role_assigner.min_num_islands);
  params_.role_assigner.min_num_trails = std::max(0, params_.role_assigner.min_num_trails);
  params_.collector.min_dist_collision = std::max(0.1, params_.collector.min_dist_collision);
  params_.collector.min_vel = std::max(0.0, params_.collector.min_vel);
  params_.collector.velocity_factor = std::max(0.0, params_.collector.velocity_factor);
  params_.potential_field.d0 = std::max(0.1, params_.potential_field.d0);
  params_.potential_field.df = std::max(0.1, params_.potential_field.df);
  params_.potential_field.dc = std::max(0.1, params_.potential_field.dc);
  params_.racer_visualizer.occupied_threshold =
    std::max(1, params_.racer_visualizer.occupied_threshold);
  params_.racer_visualizer.visualization_period =
    std::max(0.1, params_.racer_visualizer.visualization_period);
  params_.racer_visualizer.unknown_downsample =
    std::max(1, params_.racer_visualizer.unknown_downsample);
  params_.racer_visualizer.explored_downsample =
    std::max(1, params_.racer_visualizer.explored_downsample);
  params_.racer_visualizer.occupied_downsample =
    std::max(1, params_.racer_visualizer.occupied_downsample);
}

void ExplorationGoalManager::createGoalPublishers()
{
  goal_publishers_.reserve(static_cast<size_t>(params_.drone_num));

  for (int drone_id = 0; drone_id < params_.drone_num; ++drone_id)
  {
    const std::string topic =
      "/drone_" + std::to_string(drone_id) + "_planning/exploration_goal";
    goal_publishers_.push_back(
      node_.create_publisher<geometry_msgs::msg::PoseStamped>(topic, 1));
    RCLCPP_INFO(
      node_.get_logger(), "Created exploration goal publisher for drone %d on %s",
      drone_id, topic.c_str());
  }
}

void ExplorationGoalManager::createOdometrySubscriptions()
{
  latest_odoms_.resize(static_cast<size_t>(params_.drone_num));
  odom_subs_.reserve(static_cast<size_t>(params_.drone_num));

  for (int drone_id = 0; drone_id < params_.drone_num; ++drone_id)
  {
    std::string suffix = params_.odom_topic_suffix;
    if (!suffix.empty() && suffix.front() != '/' && suffix.front() != '_')
    {
      suffix = "_" + suffix;
    }

    const std::string topic =
      params_.odom_topic_prefix + std::to_string(drone_id) + suffix;
    odom_subs_.push_back(node_.create_subscription<nav_msgs::msg::Odometry>(
      topic, 20,
      [this, drone_id](const nav_msgs::msg::Odometry::SharedPtr msg) {
        onOdometry(drone_id, msg);
      }));

    RCLCPP_INFO(
      node_.get_logger(), "Subscribed to odometry for drone %d on %s",
      drone_id, topic.c_str());
  }
}

void ExplorationGoalManager::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_map_ = *msg;
  has_map_ = true;
}

void ExplorationGoalManager::onOdometry(
  const int drone_id, const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (drone_id < 0 || drone_id >= static_cast<int>(latest_odoms_.size()))
  {
    return;
  }

  latest_odoms_[static_cast<size_t>(drone_id)] = msg;
}

void ExplorationGoalManager::processLatestMap()
{
  if (!has_map_)
  {
    return;
  }

  frontier_detector_.setMap(latest_map_);
  viewpoint_sampler_.setMap(latest_map_);
  racer_map_visualizer_.setStamp(node_.now());
  racer_map_visualizer_.setMap(latest_map_);

  auto clusters = frontier_detector_.detectFrontiers();
  if (clusters.empty() && params_.frontier.cluster_min > 10)
  {
    FrontierDetector relaxed_frontier_detector;
    auto relaxed_frontier_params = params_.frontier;
    relaxed_frontier_params.cluster_min = 10;
    relaxed_frontier_detector.setParameters(relaxed_frontier_params, params_.default_goal_z);
    relaxed_frontier_detector.setMap(latest_map_);
    clusters = relaxed_frontier_detector.detectFrontiers();
    if (!clusters.empty())
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "No frontiers passed cluster_min=%d; using relaxed cluster_min=%d with %zu clusters.",
        params_.frontier.cluster_min,
        relaxed_frontier_params.cluster_min,
        clusters.size());
    }
  }

  int unknown_count = 0;
  int free_count = 0;
  int occupied_count = 0;
  for (const auto cell : latest_map_.data)
  {
    if (cell < 0)
    {
      ++unknown_count;
    }
    else if (cell >= 50)
    {
      ++occupied_count;
    }
    else
    {
      ++free_count;
    }
  }
  RCLCPP_INFO_THROTTLE(
    node_.get_logger(), *node_.get_clock(), 3000,
    "Exploration map cells: unknown=%d free=%d occupied=%d",
    unknown_count, free_count, occupied_count);
  RCLCPP_INFO(node_.get_logger(), "Detected %zu frontier clusters.", clusters.size());
  visualizer_.publishFrontiers(clusters);

  std::vector<Eigen::Vector3d> drone_positions;
  std::vector<Eigen::Vector3d> drone_velocities;
  drone_positions.reserve(static_cast<size_t>(params_.drone_num));
  drone_velocities.reserve(static_cast<size_t>(params_.drone_num));
  for (int drone_id = 0; drone_id < params_.drone_num; ++drone_id)
  {
    const auto index = static_cast<size_t>(drone_id);
    if (index < latest_odoms_.size() && latest_odoms_[index])
    {
      drone_positions.emplace_back(
        latest_odoms_[index]->pose.pose.position.x,
        latest_odoms_[index]->pose.pose.position.y,
        latest_odoms_[index]->pose.pose.position.z);
      drone_velocities.emplace_back(
        latest_odoms_[index]->twist.twist.linear.x,
        latest_odoms_[index]->twist.twist.linear.y,
        latest_odoms_[index]->twist.twist.linear.z);
    }
    else
    {
      drone_positions.emplace_back(Eigen::Vector3d::Zero());
      drone_velocities.emplace_back(Eigen::Vector3d::Zero());
    }
  }
  std::map<int, std::vector<int>> hgrid_assignment;
  if (params_.enable_hgrid)
  {
    hgrid_allocator_.setMap(latest_map_);
    hgrid_allocator_.update(clusters);
    hgrid_assignment =
      hgrid_allocator_.assignGridsToDrones(drone_positions, drone_velocities, previous_grid_ids_);
    visualizer_.publishHGrid(hgrid_allocator_.getGridCells(), hgrid_assignment);
  }
  else
  {
    visualizer_.clearHGrid();
  }

  if (params_.racer_visualizer.enable_racer_visualizer)
  {
    if (params_.racer_visualizer.publish_explored_cloud && explored_area_pub_)
    {
      explored_area_pub_->publish(racer_map_visualizer_.buildExploredCloud());
    }
    if (params_.racer_visualizer.publish_occupied_cloud && occupied_area_pub_)
    {
      occupied_area_pub_->publish(racer_map_visualizer_.buildOccupiedCloud());
    }
    if (params_.racer_visualizer.publish_unknown_cloud && unknown_area_pub_)
    {
      unknown_area_pub_->publish(racer_map_visualizer_.buildUnknownCloud());
    }
    if (ego_map_adapter_)
    {
      if (params_.racer_visualizer.publish_explored_cloud && explored_area_3d_pub_)
      {
        explored_area_3d_pub_->publish(ego_map_adapter_->buildExploredCloud3D(node_.now()));
      }
      if (params_.racer_visualizer.publish_occupied_cloud && occupied_area_3d_pub_)
      {
        occupied_area_3d_pub_->publish(ego_map_adapter_->buildOccupiedCloud3D(node_.now()));
      }
    }
  }

  std::vector<Viewpoint> viewpoints;
  for (const auto & cluster : clusters)
  {
    const auto cluster_viewpoints = viewpoint_sampler_.sampleViewpoints(cluster);
    viewpoints.insert(viewpoints.end(), cluster_viewpoints.begin(), cluster_viewpoints.end());
  }

  if (viewpoints.empty() && !clusters.empty())
  {
    viewpoints = createFallbackViewpoints(clusters);
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 3000,
      "Strict viewpoint sampling produced 0 candidates; using %zu safe fallback viewpoints.",
      viewpoints.size());
  }

  RCLCPP_INFO(node_.get_logger(), "Generated %zu candidate viewpoints.", viewpoints.size());
  visualizer_.publishViewpoints(viewpoints);
  updateFrontierGoals(clusters, viewpoints, hgrid_assignment);
}

std::vector<Viewpoint> ExplorationGoalManager::createFallbackViewpoints(
  const std::vector<FrontierCluster> & clusters) const
{
  std::vector<Viewpoint> viewpoints;
  if (!has_map_)
  {
    return viewpoints;
  }

  nav_msgs::msg::Odometry::SharedPtr odom;
  for (const auto & candidate_odom : latest_odoms_)
  {
    if (candidate_odom)
    {
      odom = candidate_odom;
      break;
    }
  }
  if (!odom)
  {
    return viewpoints;
  }

  const Eigen::Vector3d drone_position(
    odom->pose.pose.position.x,
    odom->pose.pose.position.y,
    odom->pose.pose.position.z);

  const int ring_count = std::max(3, params_.frontier.candidate_rnum);
  const double radius_min = std::max(0.5, params_.frontier.candidate_rmin);
  const double radius_max = std::max(radius_min, params_.explorer.ftr_max_distance);
  const double radius_span = radius_max - radius_min;
  const double angular_step = std::max(params_.frontier.candidate_dphi, 0.2617993878);

  for (const auto & cluster : clusters)
  {
    const Eigen::Vector3d center = cluster.average;
    const auto & cells = cluster.filtered_cells.empty() ? cluster.cells : cluster.filtered_cells;
    const int visible_hint = std::max(1, static_cast<int>(cells.size()));

    for (int ring = 0; ring < ring_count; ++ring)
    {
      const double ratio =
        ring_count == 1 ? 0.0 : static_cast<double>(ring) / static_cast<double>(ring_count - 1);
      const double radius = radius_min + radius_span * ratio;

      for (double phi = 0.0; phi < 2.0 * M_PI; phi += angular_step)
      {
        Eigen::Vector3d candidate(
          center.x() + radius * std::cos(phi),
          center.y() + radius * std::sin(phi),
          params_.default_goal_z);

        if (!isPositionSafe(candidate))
        {
          continue;
        }

        const double drone_distance = (candidate - drone_position).norm();
        if (drone_distance < params_.min_goal_distance ||
          drone_distance > params_.max_goal_distance)
        {
          continue;
        }

        Viewpoint viewpoint;
        viewpoint.position = candidate;
        viewpoint.yaw = std::atan2(center.y() - candidate.y(), center.x() - candidate.x());
        viewpoint.visible_num = visible_hint;
        viewpoint.frontier_id = cluster.id;
        viewpoints.push_back(viewpoint);
      }
    }
  }

  return viewpoints;
}

void ExplorationGoalManager::updateFrontierGoals(
  const std::vector<FrontierCluster> & frontiers,
  const std::vector<Viewpoint> & viewpoints,
  const std::map<int, std::vector<int>> & hgrid_assignment)
{
  if (goal_publishers_.empty())
  {
    return;
  }

  bool any_odom = false;
  for (const auto & odom : latest_odoms_)
  {
    any_odom = any_odom || static_cast<bool>(odom);
  }
  if (!any_odom)
  {
    RCLCPP_INFO_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 3000,
      "Waiting for drone odometry before publishing frontier goals.");
    return;
  }

  std::vector<DroneState> swarm_states;
  swarm_states.reserve(static_cast<size_t>(params_.drone_num));
  for (int drone_id = 0; drone_id < params_.drone_num; ++drone_id)
  {
    const auto drone_index = static_cast<size_t>(drone_id);
    DroneState state;
    state.id = drone_id;
    if (drone_index < latest_odoms_.size() && latest_odoms_[drone_index])
    {
      state.position = Eigen::Vector3d(
        latest_odoms_[drone_index]->pose.pose.position.x,
        latest_odoms_[drone_index]->pose.pose.position.y,
        latest_odoms_[drone_index]->pose.pose.position.z);
      state.velocity = Eigen::Vector3d(
        latest_odoms_[drone_index]->twist.twist.linear.x,
        latest_odoms_[drone_index]->twist.twist.linear.y,
        latest_odoms_[drone_index]->twist.twist.linear.z);
    }
    if (drone_index < has_current_goals_.size() && has_current_goals_[drone_index])
    {
      state.has_goal = true;
      state.current_goal = Eigen::Vector3d(
        current_goals_[drone_index].pose.position.x,
        current_goals_[drone_index].pose.position.y,
        current_goals_[drone_index].pose.position.z);
    }
    if (drone_index < current_roles_.size())
    {
      state.role = current_roles_[drone_index];
    }
    swarm_states.push_back(state);
  }

  for (auto & state : swarm_states)
  {
    const auto drone_index = static_cast<size_t>(state.id);
    if (drone_index >= latest_odoms_.size() || !latest_odoms_[drone_index])
    {
      state.role = Role::UNKNOWN;
      continue;
    }
    if (params_.enable_role_assignment)
    {
      state.role = role_assigner_.assignRole(
        state.id, state.position, state.velocity, swarm_states, frontiers);
    }
    else
    {
      state.role = Role::EXPLORER;
    }
    current_roles_[drone_index] = state.role;
    RCLCPP_INFO_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 3000,
      "Drone %d role: %s", state.id, roleToString(state.role));
  }
  visualizer_.publishRoles(swarm_states);

  if (viewpoints.empty())
  {
    RCLCPP_INFO_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 3000,
      "No valid frontier viewpoint available for swarm assignment.");
    std::vector<geometry_msgs::msg::PoseStamped> active_goals;
    for (int drone_id = 0; drone_id < params_.drone_num; ++drone_id)
    {
      if (has_current_goals_[static_cast<size_t>(drone_id)])
      {
        active_goals.push_back(current_goals_[static_cast<size_t>(drone_id)]);
      }
    }
    visualizer_.publishCurrentGoals(active_goals);
    return;
  }

  const int max_visible_num = std::max_element(
    viewpoints.begin(), viewpoints.end(),
    [](const Viewpoint & lhs, const Viewpoint & rhs) {
      return lhs.visible_num < rhs.visible_num;
    })->visible_num;

  std::vector<int> assigned_indices;
  assigned_indices.reserve(static_cast<size_t>(params_.drone_num));
  std::vector<geometry_msgs::msg::PoseStamped> assigned_goals;
  assigned_goals.reserve(static_cast<size_t>(params_.drone_num));
  for (int drone_id = 0; drone_id < params_.drone_num; ++drone_id)
  {
    const auto drone_index = static_cast<size_t>(drone_id);
    if (params_.enable_multi_drone_separation &&
      drone_index < has_current_goals_.size() && has_current_goals_[drone_index] &&
      isGoalValid(current_goals_[drone_index]) && !shouldSelectNewGoal(drone_id))
    {
      assigned_goals.push_back(current_goals_[drone_index]);
    }
  }

  for (int drone_id = 0; drone_id < params_.drone_num; ++drone_id)
  {
    const auto drone_index = static_cast<size_t>(drone_id);
    if (drone_index >= latest_odoms_.size() || !latest_odoms_[drone_index])
    {
      RCLCPP_INFO_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 3000,
        "Waiting for drone %d odometry before assigning frontier goal.", drone_id);
      continue;
    }

    if (has_current_goals_[drone_index] && !isGoalValid(current_goals_[drone_index]))
    {
      has_current_goals_[drone_index] = false;
    }

    if (!shouldSelectNewGoal(drone_id))
    {
      continue;
    }

    int best_index = -1;
    double best_score = std::numeric_limits<double>::infinity();
    const Eigen::Vector3d drone_position(
      latest_odoms_[drone_index]->pose.pose.position.x,
      latest_odoms_[drone_index]->pose.pose.position.y,
      latest_odoms_[drone_index]->pose.pose.position.z);

    const auto assigned_frontier_ids =
      params_.enable_hgrid ?
      hgrid_allocator_.getFrontiersInAssignedGrids(drone_id, hgrid_assignment) :
      std::vector<int>{};
    const std::set<int> assigned_frontier_set(
      assigned_frontier_ids.begin(), assigned_frontier_ids.end());
    std::vector<int> assigned_grid_ids;
    const auto assigned_grid_it = hgrid_assignment.find(drone_id);
    if (assigned_grid_it != hgrid_assignment.end())
    {
      assigned_grid_ids = assigned_grid_it->second;
    }

    for (int pass = 0; pass < 2 && best_index < 0; ++pass)
    {
      const bool has_assigned_hgrid = !assigned_frontier_set.empty() || !assigned_grid_ids.empty();
      const bool prefer_assigned_grid = pass == 0 && has_assigned_hgrid;
      if (pass == 0 && !has_assigned_hgrid)
      {
        continue;
      }
      if (pass == 1 && params_.enable_hgrid)
      {
        RCLCPP_INFO_THROTTLE(
          node_.get_logger(), *node_.get_clock(), 3000,
          "Drone %d has no valid candidate in assigned HGrid; falling back to all frontiers.",
          drone_id);
      }

      for (size_t i = 0; i < viewpoints.size(); ++i)
      {
        if (params_.enable_multi_drone_separation &&
          std::find(assigned_indices.begin(), assigned_indices.end(), static_cast<int>(i)) !=
          assigned_indices.end())
        {
          continue;
        }

        const auto & viewpoint = viewpoints[i];
        if (prefer_assigned_grid &&
          assigned_frontier_set.find(viewpoint.frontier_id) == assigned_frontier_set.end() &&
          !isViewpointInAssignedHGrid(viewpoint, assigned_grid_ids))
        {
          continue;
        }
        if (!isPositionSafe(viewpoint.position))
        {
          continue;
        }

        const double distance = (viewpoint.position - drone_position).norm();
        if (distance < params_.min_goal_distance || distance > params_.max_goal_distance)
        {
          continue;
        }

        if (params_.enable_multi_drone_separation &&
          !isSeparatedFromAssignedGoals(viewpoint, assigned_goals))
        {
          continue;
        }

        const double score =
          current_roles_[drone_index] == Role::GARBAGE_COLLECTOR ?
          scoreCollectorViewpoint(drone_id, viewpoint, assigned_goals) :
          scoreViewpoint(drone_id, viewpoint, max_visible_num, assigned_goals);
        if (score < best_score)
        {
          best_score = score;
          best_index = static_cast<int>(i);
        }
      }
    }

    if (best_index < 0)
    {
      RCLCPP_INFO_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 3000,
        "No frontier viewpoint passed assignment checks for drone %d.", drone_id);
      continue;
    }

    if (has_current_goals_[drone_index])
    {
      previous_goals_[drone_index] = current_goals_[drone_index];
      has_previous_goals_[drone_index] = true;
    }

    current_goals_[drone_index] = viewpointToGoal(viewpoints[static_cast<size_t>(best_index)]);
    current_goal_stamps_[drone_index] = node_.now();
    has_current_goals_[drone_index] = true;
    if (params_.enable_multi_drone_separation)
    {
      assigned_indices.push_back(best_index);
      assigned_goals.push_back(current_goals_[drone_index]);
    }
    if (params_.enable_hgrid && !assigned_grid_ids.empty())
    {
      previous_grid_ids_[drone_index] = assigned_grid_ids.front();
    }

    goal_publishers_[drone_index]->publish(current_goals_[drone_index]);

    const auto & goal = current_goals_[drone_index];
    const auto & viewpoint = viewpoints[static_cast<size_t>(best_index)];
    RCLCPP_INFO(
      node_.get_logger(),
      "Drone %d role=%s selected frontier_id=%d, goal=[%.2f,%.2f,%.2f], score=%.3f",
      drone_id,
      roleToString(current_roles_[drone_index]),
      viewpoint.frontier_id,
      goal.pose.position.x,
      goal.pose.position.y,
      goal.pose.position.z,
      best_score);
  }

  std::vector<geometry_msgs::msg::PoseStamped> active_goals;
  for (int drone_id = 0; drone_id < params_.drone_num; ++drone_id)
  {
    if (has_current_goals_[static_cast<size_t>(drone_id)])
    {
      active_goals.push_back(current_goals_[static_cast<size_t>(drone_id)]);
    }
  }
  visualizer_.publishCurrentGoals(active_goals);
}

bool ExplorationGoalManager::shouldSelectNewGoal(const int drone_id) const
{
  const auto index = static_cast<size_t>(drone_id);
  if (index >= has_current_goals_.size() || !has_current_goals_[index])
  {
    return true;
  }

  if (!params_.publish_only_when_reached)
  {
    return true;
  }

  return isCurrentGoalReached(drone_id) || isCurrentGoalTimedOut(drone_id) ||
         !isGoalValid(current_goals_[index]);
}

bool ExplorationGoalManager::isCurrentGoalReached(const int drone_id) const
{
  const auto index = static_cast<size_t>(drone_id);
  if (index >= has_current_goals_.size() || index >= latest_odoms_.size() ||
    !has_current_goals_[index] || !latest_odoms_[index])
  {
    return false;
  }

  const auto & position = latest_odoms_[index]->pose.pose.position;
  const double distance =
    std::hypot(
    current_goals_[index].pose.position.x - position.x,
    current_goals_[index].pose.position.y - position.y);
  const double dz = current_goals_[index].pose.position.z - position.z;
  return std::hypot(distance, dz) <= params_.goal_reached_threshold;
}

bool ExplorationGoalManager::isCurrentGoalTimedOut(const int drone_id) const
{
  const auto index = static_cast<size_t>(drone_id);
  if (index >= has_current_goals_.size() || !has_current_goals_[index])
  {
    return false;
  }

  return (node_.now() - current_goal_stamps_[index]).seconds() >= params_.goal_timeout;
}

bool ExplorationGoalManager::isGoalValid(const geometry_msgs::msg::PoseStamped & goal) const
{
  if (!has_map_)
  {
    return false;
  }

  int mx = 0;
  int my = 0;
  if (!worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my))
  {
    return false;
  }

  const auto index = static_cast<size_t>(my * static_cast<int>(latest_map_.info.width) + mx);
  if (index >= latest_map_.data.size())
  {
    return false;
  }

  return latest_map_.data[index] >= 0 && latest_map_.data[index] < 50 &&
         isPositionSafe(
    Eigen::Vector3d(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z));
}

bool ExplorationGoalManager::worldToMap(
  const double x, const double y, int & mx, int & my) const
{
  if (latest_map_.info.resolution <= 0.0)
  {
    return false;
  }

  mx = static_cast<int>(
    std::floor((x - latest_map_.info.origin.position.x) / latest_map_.info.resolution));
  my = static_cast<int>(
    std::floor((y - latest_map_.info.origin.position.y) / latest_map_.info.resolution));
  return mx >= 0 && my >= 0 &&
         mx < static_cast<int>(latest_map_.info.width) &&
         my < static_cast<int>(latest_map_.info.height);
}

bool ExplorationGoalManager::isMapCellFree(const int mx, const int my) const
{
  if (!has_map_ || mx < 0 || my < 0 ||
    mx >= static_cast<int>(latest_map_.info.width) ||
    my >= static_cast<int>(latest_map_.info.height))
  {
    return false;
  }

  const auto index = static_cast<size_t>(my * static_cast<int>(latest_map_.info.width) + mx);
  return index < latest_map_.data.size() && latest_map_.data[index] == 0;
}

bool ExplorationGoalManager::isPositionSafe(const Eigen::Vector3d & position) const
{
  if (!has_map_ || latest_map_.info.resolution <= 0.0)
  {
    return false;
  }

  int center_mx = 0;
  int center_my = 0;
  if (!worldToMap(position.x(), position.y(), center_mx, center_my))
  {
    return false;
  }

  if (!isMapCellFree(center_mx, center_my))
  {
    return false;
  }

  const int radius_cells =
    static_cast<int>(std::ceil(params_.frontier.min_candidate_clearance / latest_map_.info.resolution));

  for (int dy = -radius_cells; dy <= radius_cells; ++dy)
  {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx)
    {
      const int mx = center_mx + dx;
      const int my = center_my + dy;
      if (mx < 0 || my < 0 ||
        mx >= static_cast<int>(latest_map_.info.width) ||
        my >= static_cast<int>(latest_map_.info.height))
      {
        return false;
      }

      const double cell_x =
        latest_map_.info.origin.position.x +
        (static_cast<double>(mx) + 0.5) * latest_map_.info.resolution;
      const double cell_y =
        latest_map_.info.origin.position.y +
        (static_cast<double>(my) + 0.5) * latest_map_.info.resolution;
      const double distance = std::hypot(cell_x - position.x(), cell_y - position.y());
      if (distance <= params_.frontier.min_candidate_clearance && !isMapCellFree(mx, my))
      {
        return false;
      }
    }
  }

  return true;
}

bool ExplorationGoalManager::isSeparatedFromAssignedGoals(
  const Viewpoint & viewpoint,
  const std::vector<geometry_msgs::msg::PoseStamped> & assigned_goals) const
{
  for (const auto & goal : assigned_goals)
  {
    const Eigen::Vector3d assigned_position(
      goal.pose.position.x,
      goal.pose.position.y,
      goal.pose.position.z);
    if ((viewpoint.position - assigned_position).norm() < params_.min_goal_separation)
    {
      return false;
    }
  }

  return true;
}

bool ExplorationGoalManager::isViewpointInAssignedHGrid(
  const Viewpoint & viewpoint,
  const std::vector<int> & assigned_grid_ids) const
{
  if (assigned_grid_ids.empty())
  {
    return false;
  }

  for (const auto & grid : hgrid_allocator_.getGridCells())
  {
    if (std::find(assigned_grid_ids.begin(), assigned_grid_ids.end(), grid.id) ==
      assigned_grid_ids.end())
    {
      continue;
    }

    const double half_size = 0.5 * grid.size;
    if (viewpoint.position.x() >= grid.center.x() - half_size &&
      viewpoint.position.x() < grid.center.x() + half_size &&
      viewpoint.position.y() >= grid.center.y() - half_size &&
      viewpoint.position.y() < grid.center.y() + half_size)
    {
      return true;
    }
  }

  return false;
}

double ExplorationGoalManager::scoreViewpoint(
  const int drone_id,
  const Viewpoint & viewpoint,
  const int max_visible_num,
  const std::vector<geometry_msgs::msg::PoseStamped> & assigned_goals) const
{
  const auto index = static_cast<size_t>(drone_id);
  const auto & odom = latest_odoms_[index];
  const Eigen::Vector3d drone_position(
    odom->pose.pose.position.x,
    odom->pose.pose.position.y,
    odom->pose.pose.position.z);

  const double distance_cost = (viewpoint.position - drone_position).norm();
  const double direction_cost =
    std::abs(normalizeAngle(viewpoint.yaw - currentYaw(drone_id))) /
    params_.explorer.max_ang_dist;

  double previous_goal_cost = 0.0;
  if (has_previous_goals_[index])
  {
    const Eigen::Vector3d previous_position(
      previous_goals_[index].pose.position.x,
      previous_goals_[index].pose.position.y,
      previous_goals_[index].pose.position.z);
    const double previous_distance = (viewpoint.position - previous_position).norm();
    previous_goal_cost = previous_distance < params_.min_goal_distance ? 1.0 : 0.0;
  }

  double other_drone_cost = 0.0;
  double collaboration_cost = 0.0;
  if (params_.enable_multi_drone_separation)
  {
    for (int other_id = 0; other_id < params_.drone_num; ++other_id)
    {
      if (other_id == drone_id)
      {
        continue;
      }

      const auto other_index = static_cast<size_t>(other_id);
      if (other_index < latest_odoms_.size() && latest_odoms_[other_index])
      {
        const Eigen::Vector3d other_position(
          latest_odoms_[other_index]->pose.pose.position.x,
          latest_odoms_[other_index]->pose.pose.position.y,
          latest_odoms_[other_index]->pose.pose.position.z);
        const double dist_to_other = (viewpoint.position - other_position).norm();
        if (dist_to_other < params_.collaboration.communication_range)
        {
          other_drone_cost +=
            std::max(0.0, params_.collaboration.dist_collision - dist_to_other) /
            params_.collaboration.dist_collision;
          if (params_.collaboration.active && params_.collaboration.active_collaboration)
          {
            collaboration_cost +=
              params_.collaboration.w_collision *
              std::max(0.0, params_.collaboration.dist_collision - dist_to_other) /
              params_.collaboration.dist_collision;
            collaboration_cost +=
              params_.collaboration.w_range *
              std::max(0.0, params_.collaboration.dist_range - dist_to_other) /
              params_.collaboration.dist_range;
          }
        }
      }

      if (other_index < has_current_goals_.size() && has_current_goals_[other_index])
      {
        const Eigen::Vector3d other_goal(
          current_goals_[other_index].pose.position.x,
          current_goals_[other_index].pose.position.y,
          current_goals_[other_index].pose.position.z);
        const double dist_to_goal = (viewpoint.position - other_goal).norm();
        other_drone_cost +=
          std::max(0.0, params_.min_goal_separation - dist_to_goal) /
          params_.min_goal_separation;
      }
    }

    for (const auto & assigned_goal : assigned_goals)
    {
      const Eigen::Vector3d assigned_position(
        assigned_goal.pose.position.x,
        assigned_goal.pose.position.y,
        assigned_goal.pose.position.z);
      const double dist_to_assigned = (viewpoint.position - assigned_position).norm();
      other_drone_cost +=
        std::max(0.0, params_.min_goal_separation - dist_to_assigned) /
        params_.min_goal_separation;
    }
  }

  const double information_gain_cost =
    max_visible_num > 0 ? static_cast<double>(viewpoint.visible_num) / max_visible_num : 0.0;

  return params_.explorer.w_distance * distance_cost +
         params_.explorer.w_direction * direction_cost +
         params_.explorer.w_others * other_drone_cost +
         params_.explorer.w_previous_goal * previous_goal_cost +
         collaboration_cost -
         information_gain_cost;
}

double ExplorationGoalManager::scoreCollectorViewpoint(
  const int drone_id,
  const Viewpoint & viewpoint,
  const std::vector<geometry_msgs::msg::PoseStamped> & assigned_goals) const
{
  const auto index = static_cast<size_t>(drone_id);
  const auto & odom = latest_odoms_[index];
  const Eigen::Vector3d drone_position(
    odom->pose.pose.position.x,
    odom->pose.pose.position.y,
    odom->pose.pose.position.z);
  const Eigen::Vector3d drone_velocity(
    odom->twist.twist.linear.x,
    odom->twist.twist.linear.y,
    odom->twist.twist.linear.z);

  const double velocity_scale =
    1.0 + params_.collector.velocity_factor *
    std::max(0.0, drone_velocity.norm() - params_.collector.min_vel);
  const double distance_cost = (viewpoint.position - drone_position).norm() / velocity_scale;
  const double direction_cost =
    std::abs(normalizeAngle(viewpoint.yaw - currentYaw(drone_id))) /
    params_.explorer.max_ang_dist;

  double previous_goal_cost = 0.0;
  if (has_previous_goals_[index])
  {
    const Eigen::Vector3d previous_position(
      previous_goals_[index].pose.position.x,
      previous_goals_[index].pose.position.y,
      previous_goals_[index].pose.position.z);
    const double previous_distance = (viewpoint.position - previous_position).norm();
    previous_goal_cost = previous_distance < params_.min_goal_distance ? 1.0 : 0.0;
  }

  double other_drone_cost = 0.0;
  if (params_.enable_multi_drone_separation)
  {
    for (int other_id = 0; other_id < params_.drone_num; ++other_id)
    {
      if (other_id == drone_id)
      {
        continue;
      }

      const auto other_index = static_cast<size_t>(other_id);
      if (other_index < latest_odoms_.size() && latest_odoms_[other_index])
      {
        const Eigen::Vector3d other_position(
          latest_odoms_[other_index]->pose.pose.position.x,
          latest_odoms_[other_index]->pose.pose.position.y,
          latest_odoms_[other_index]->pose.pose.position.z);
        const double dist_to_other = (viewpoint.position - other_position).norm();
        other_drone_cost +=
          std::max(0.0, params_.collector.min_dist_collision - dist_to_other) /
          params_.collector.min_dist_collision;
      }
    }

    for (const auto & assigned_goal : assigned_goals)
    {
      const Eigen::Vector3d assigned_position(
        assigned_goal.pose.position.x,
        assigned_goal.pose.position.y,
        assigned_goal.pose.position.z);
      const double dist_to_assigned = (viewpoint.position - assigned_position).norm();
      other_drone_cost +=
        std::max(0.0, params_.min_goal_separation - dist_to_assigned) /
        params_.min_goal_separation;
    }
  }

  const double label_cost =
    std::min(
    1.0,
    static_cast<double>(std::max(0, viewpoint.visible_num)) /
    static_cast<double>(std::max(1, params_.frontier.min_visib_num)));

  return params_.collector.w_distance * distance_cost +
         params_.collector.w_direction * direction_cost +
         params_.collector.w_others * other_drone_cost +
         params_.collector.w_previous_goal * previous_goal_cost +
         params_.collector.label_penalty * label_cost;
}

geometry_msgs::msg::PoseStamped ExplorationGoalManager::viewpointToGoal(
  const Viewpoint & viewpoint) const
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = node_.now();
  goal.header.frame_id = params_.frame_id;
  goal.pose.position.x = viewpoint.position.x();
  goal.pose.position.y = viewpoint.position.y();
  goal.pose.position.z = params_.default_goal_z;
  goal.pose.orientation.z = std::sin(viewpoint.yaw * 0.5);
  goal.pose.orientation.w = std::cos(viewpoint.yaw * 0.5);
  return goal;
}

double ExplorationGoalManager::currentYaw(const int drone_id) const
{
  const auto index = static_cast<size_t>(drone_id);
  if (index >= latest_odoms_.size() || !latest_odoms_[index])
  {
    return 0.0;
  }

  const auto & q = latest_odoms_[index]->pose.pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double ExplorationGoalManager::normalizeAngle(double angle)
{
  while (angle > M_PI)
  {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI)
  {
    angle += 2.0 * M_PI;
  }
  return angle;
}

void ExplorationGoalManager::onTimer()
{
  if (has_map_)
  {
    processLatestMap();
    return;
  }

  RCLCPP_INFO_THROTTLE(
    node_.get_logger(), *node_.get_clock(), 10000,
    "Waiting for map and odometry for frontier-based exploration.");
}

}  // namespace exploration_swarm
