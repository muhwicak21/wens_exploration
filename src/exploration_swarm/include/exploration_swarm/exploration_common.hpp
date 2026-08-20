#pragma once

#include <string>
#include <vector>

#include "Eigen/Core"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace exploration_swarm
{

enum class FrontierLabel
{
  TRAIL = 0,
  FRONTIER = 1,
  UNLABELED = 2
};

struct FrontierCluster
{
  int id{0};
  FrontierLabel label{FrontierLabel::UNLABELED};
  std::vector<Eigen::Vector3d> cells;
  std::vector<Eigen::Vector3d> filtered_cells;
  Eigen::Vector3d average{Eigen::Vector3d::Zero()};
  Eigen::Vector3d box_min{Eigen::Vector3d::Zero()};
  Eigen::Vector3d box_max{Eigen::Vector3d::Zero()};
};

struct FrontierParameters
{
  int cluster_min{100};
  double cluster_size_xy{2.0};
  double cluster_size_z{10.0};
  double min_candidate_dist{0.5};
  double min_candidate_clearance{0.41};
  double candidate_dphi{0.2617993878};
  int candidate_rnum{3};
  double candidate_rmin{1.0};
  double candidate_rmax{1.5};
  int down_sample{1};
  int min_visib_num{30};
  double min_view_finish_fraction{0.2};
};

struct PerceptionParameters
{
  double top_angle{0.56125};
  double left_angle{0.69222};
  double right_angle{0.68901};
  double max_dist{4.5};
  double vis_dist{1.0};
};

struct RacerVisualizerParameters
{
  int occupied_threshold{50};
  int free_value{0};
  int unknown_value{-1};
  bool enable_racer_visualizer{true};
  bool publish_explored_cloud{true};
  bool publish_occupied_cloud{true};
  bool publish_unknown_cloud{true};
  double visualization_period{0.5};
  double visualization_z{0.05};
  int unknown_downsample{2};
  int explored_downsample{1};
  int occupied_downsample{1};
};

struct ExplorationTuningParameters
{
  bool refine_local{true};
  int refined_num{7};
  double refined_radius{5.0};
  double max_decay{0.8};
  int top_view_num{15};
  double vm{1.5};
  double am{1.5};
  double yd{1.3962634016};
  double ydd{1.5707963268};
  double w_dir{1.5};
  int init_plan_num{2};
};

struct ExplorerParameters
{
  double ftr_max_distance{6.0};
  double max_ang_dist{0.5235987756};
  double label_penalty{0.0};
  double w_distance{0.0};
  double w_direction{10.0};
  double w_others{1.0};
  double w_previous_goal{1.0};
};

struct CollaborationParameters
{
  double communication_range{100000.0};
  bool active_collaboration{true};
  double dist_range{15.0};
  double dist_collision{5.0};
  double w_range{0.7};
  double w_collision{0.3};
  bool active{true};
};

struct PartitioningParameters
{
  int min_unknown{4000};
  int min_frontier{100};
  int min_free{3000};
  double consistent_cost{-5.0};
  double consistent_cost2{8.0};
  double w_unknown{0.0};
  double grid_size{15.0};
  bool use_swarm_tf{true};
};

struct RoleParams
{
  double region_size{8.0};
  double cluster_xy_size{3.0};
  int min_num_neighbours{3};
  int min_num_islands{4};
  int min_num_trails{1};
  bool fixed{false};
  int fix_role{0};
};

struct CollectorParameters
{
  double min_dist_collision{5.0};
  double min_vel{0.7};
  double label_penalty{15.0};
  double velocity_factor{2.0};
  double w_distance{3.0};
  double w_direction{1.0};
  double w_others{1.0};
  double w_previous_goal{3.0};
};

struct PotentialFieldParameters
{
  double ka{1.0};
  double kr{1.0};
  double d0{5.0};
  double df{6.0};
  double dc{2.0};
};

struct ExplorationParameters
{
  int drone_num{4};
  std::string map_source_type{"pointcloud"};
  std::string map_topic{"/swarm_exploration/occupancy_grid"};
  std::string odom_topic_prefix{"/drone_"};
  std::string odom_topic_suffix{"visual_slam/odom"};
  std::string frame_id{"world"};
  double map_size_x{42.0};
  double map_size_y{30.0};
  double map_size_z{5.0};
  double map_margin{1.0};
  double min_z{0.8};
  double default_goal_z{1.0};
  double publish_period{0.5};
  bool publish_only_when_reached{true};
  double goal_reached_threshold{0.8};
  double goal_timeout{20.0};
  double min_goal_distance{1.0};
  double max_goal_distance{100.0};
  double min_goal_separation{3.0};
  bool enable_multi_drone_separation{true};
  bool enable_hgrid{true};
  bool enable_role_assignment{true};
  FrontierParameters frontier;
  PerceptionParameters perception;
  RacerVisualizerParameters racer_visualizer;
  ExplorationTuningParameters exploration;
  ExplorerParameters explorer;
  CollaborationParameters collaboration;
  PartitioningParameters partitioning;
  RoleParams role_assigner;
  CollectorParameters collector;
  PotentialFieldParameters potential_field;
};

using ExplorationGoal = geometry_msgs::msg::PoseStamped;

}  // namespace exploration_swarm
