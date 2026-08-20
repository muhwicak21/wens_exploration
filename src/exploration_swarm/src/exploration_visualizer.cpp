#include "exploration_swarm/exploration_visualizer.hpp"

#include <algorithm>
#include <cmath>

#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace exploration_swarm
{

ExplorationVisualizer::ExplorationVisualizer(rclcpp::Node & node) : node_(node)
{
  frontier_marker_pub_ =
    node_.create_publisher<visualization_msgs::msg::MarkerArray>(
    "/swarm_exploration/frontiers", 1);
  viewpoint_marker_pub_ =
    node_.create_publisher<visualization_msgs::msg::MarkerArray>(
    "/swarm_exploration/viewpoints", 1);
  current_goal_marker_pub_ =
    node_.create_publisher<visualization_msgs::msg::MarkerArray>(
    "/swarm_exploration/current_goal", 1);
  current_goals_marker_pub_ =
    node_.create_publisher<visualization_msgs::msg::MarkerArray>(
    "/swarm_exploration/current_goals", 1);
  hgrid_marker_pub_ =
    node_.create_publisher<visualization_msgs::msg::MarkerArray>(
    "/swarm_exploration/hgrid", 1);
  role_marker_pub_ =
    node_.create_publisher<visualization_msgs::msg::MarkerArray>(
    "/swarm_exploration/roles", 1);
}

void ExplorationVisualizer::setFrameId(const std::string & frame_id)
{
  frame_id_ = frame_id;
}

void ExplorationVisualizer::setPerceptionParameters(const PerceptionParameters & perception)
{
  perception_ = perception;
}

void ExplorationVisualizer::publishFrontiers(const std::vector<FrontierCluster> & clusters)
{
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = frame_id_;
  clear_marker.header.stamp = node_.now();
  clear_marker.ns = "frontier_clusters";
  clear_marker.id = 0;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  int marker_id = 1;
  for (const auto & cluster : clusters)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = node_.now();
    marker.ns = "frontier_clusters";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.18;
    marker.scale.y = 0.18;
    marker.color.a = 1.0;
    marker.color.r = 0.25 + 0.65 * std::fmod(0.37 * cluster.id, 1.0);
    marker.color.g = 0.35 + 0.55 * std::fmod(0.61 * cluster.id + 0.2, 1.0);
    marker.color.b = 0.45 + 0.45 * std::fmod(0.83 * cluster.id + 0.4, 1.0);

    for (const auto & cell : cluster.filtered_cells)
    {
      geometry_msgs::msg::Point point;
      point.x = cell.x();
      point.y = cell.y();
      point.z = cell.z();
      marker.points.push_back(point);
    }

    marker_array.markers.push_back(marker);

    visualization_msgs::msg::Marker center_marker;
    center_marker.header.frame_id = frame_id_;
    center_marker.header.stamp = node_.now();
    center_marker.ns = "frontier_cluster_centers";
    center_marker.id = marker_id++;
    center_marker.type = visualization_msgs::msg::Marker::SPHERE;
    center_marker.action = visualization_msgs::msg::Marker::ADD;
    center_marker.pose.position.x = cluster.average.x();
    center_marker.pose.position.y = cluster.average.y();
    center_marker.pose.position.z = cluster.average.z();
    center_marker.pose.orientation.w = 1.0;
    center_marker.scale.x = 0.35;
    center_marker.scale.y = 0.35;
    center_marker.scale.z = 0.35;
    center_marker.color.a = 1.0;
    center_marker.color.r = 1.0;
    center_marker.color.g = 0.8;
    center_marker.color.b = 0.1;
    marker_array.markers.push_back(center_marker);
  }

  frontier_marker_pub_->publish(marker_array);
}

void ExplorationVisualizer::publishViewpoints(const std::vector<Viewpoint> & viewpoints)
{
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = frame_id_;
  clear_marker.header.stamp = node_.now();
  clear_marker.ns = "candidate_viewpoints";
  clear_marker.id = 0;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  visualization_msgs::msg::Marker points_marker;
  points_marker.header.frame_id = frame_id_;
  points_marker.header.stamp = node_.now();
  points_marker.ns = "candidate_viewpoints";
  points_marker.id = 1;
  points_marker.type = visualization_msgs::msg::Marker::POINTS;
  points_marker.action = visualization_msgs::msg::Marker::ADD;
  points_marker.pose.orientation.w = 1.0;
  points_marker.scale.x = 0.22;
  points_marker.scale.y = 0.22;
  points_marker.color.a = 1.0;
  points_marker.color.r = 0.0;
  points_marker.color.g = 0.75;
  points_marker.color.b = 1.0;

  int marker_id = 2;
  for (const auto & viewpoint : viewpoints)
  {
    geometry_msgs::msg::Point point;
    point.x = viewpoint.position.x();
    point.y = viewpoint.position.y();
    point.z = viewpoint.position.z();
    points_marker.points.push_back(point);

    visualization_msgs::msg::Marker arrow_marker;
    arrow_marker.header.frame_id = frame_id_;
    arrow_marker.header.stamp = node_.now();
    arrow_marker.ns = "candidate_viewpoint_yaws";
    arrow_marker.id = marker_id++;
    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
    arrow_marker.action = visualization_msgs::msg::Marker::ADD;
    arrow_marker.pose.position.x = viewpoint.position.x();
    arrow_marker.pose.position.y = viewpoint.position.y();
    arrow_marker.pose.position.z = viewpoint.position.z();
    arrow_marker.pose.orientation.z = std::sin(viewpoint.yaw * 0.5);
    arrow_marker.pose.orientation.w = std::cos(viewpoint.yaw * 0.5);
    arrow_marker.scale.x = std::max(0.35, perception_.vis_dist);
    arrow_marker.scale.y = 0.06;
    arrow_marker.scale.z = 0.06;
    arrow_marker.color.a = 0.85;
    arrow_marker.color.r = 0.0;
    arrow_marker.color.g = 1.0;
    arrow_marker.color.b = 0.35;
    marker_array.markers.push_back(arrow_marker);
  }

  marker_array.markers.push_back(points_marker);
  viewpoint_marker_pub_->publish(marker_array);
}

void ExplorationVisualizer::publishCurrentGoal(const geometry_msgs::msg::PoseStamped & goal)
{
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = frame_id_;
  clear_marker.header.stamp = node_.now();
  clear_marker.ns = "current_goal";
  clear_marker.id = 0;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  visualization_msgs::msg::Marker sphere_marker;
  sphere_marker.header = goal.header;
  sphere_marker.header.stamp = node_.now();
  sphere_marker.ns = "current_goal";
  sphere_marker.id = 1;
  sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
  sphere_marker.action = visualization_msgs::msg::Marker::ADD;
  sphere_marker.pose = goal.pose;
  sphere_marker.scale.x = 0.55;
  sphere_marker.scale.y = 0.55;
  sphere_marker.scale.z = 0.55;
  sphere_marker.color.a = 1.0;
  sphere_marker.color.r = 1.0;
  sphere_marker.color.g = 0.15;
  sphere_marker.color.b = 0.05;
  marker_array.markers.push_back(sphere_marker);

  visualization_msgs::msg::Marker arrow_marker;
  arrow_marker.header = goal.header;
  arrow_marker.header.stamp = node_.now();
  arrow_marker.ns = "current_goal";
  arrow_marker.id = 2;
  arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
  arrow_marker.action = visualization_msgs::msg::Marker::ADD;
  arrow_marker.pose = goal.pose;
  arrow_marker.scale.x = std::max(0.7, perception_.vis_dist);
  arrow_marker.scale.y = 0.11;
  arrow_marker.scale.z = 0.11;
  arrow_marker.color.a = 0.95;
  arrow_marker.color.r = 1.0;
  arrow_marker.color.g = 0.55;
  arrow_marker.color.b = 0.0;
  marker_array.markers.push_back(arrow_marker);

  current_goal_marker_pub_->publish(marker_array);
}

void ExplorationVisualizer::clearCurrentGoal()
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = frame_id_;
  clear_marker.header.stamp = node_.now();
  clear_marker.ns = "current_goal";
  clear_marker.id = 0;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);
  current_goal_marker_pub_->publish(marker_array);
}

void ExplorationVisualizer::publishCurrentGoals(
  const std::vector<geometry_msgs::msg::PoseStamped> & goals)
{
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = frame_id_;
  clear_marker.header.stamp = node_.now();
  clear_marker.ns = "current_swarm_goals";
  clear_marker.id = 0;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  int marker_id = 1;
  for (size_t i = 0; i < goals.size(); ++i)
  {
    const auto & goal = goals[i];

    visualization_msgs::msg::Marker sphere_marker;
    sphere_marker.header = goal.header;
    sphere_marker.header.stamp = node_.now();
    sphere_marker.ns = "current_swarm_goals";
    sphere_marker.id = marker_id++;
    sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
    sphere_marker.action = visualization_msgs::msg::Marker::ADD;
    sphere_marker.pose = goal.pose;
    sphere_marker.scale.x = 0.5;
    sphere_marker.scale.y = 0.5;
    sphere_marker.scale.z = 0.5;
    sphere_marker.color.a = 1.0;
    sphere_marker.color.r = 0.15 + 0.75 * std::fmod(0.31 * static_cast<double>(i), 1.0);
    sphere_marker.color.g = 0.25 + 0.65 * std::fmod(0.47 * static_cast<double>(i) + 0.3, 1.0);
    sphere_marker.color.b = 0.35 + 0.55 * std::fmod(0.73 * static_cast<double>(i) + 0.1, 1.0);
    marker_array.markers.push_back(sphere_marker);

    visualization_msgs::msg::Marker arrow_marker;
    arrow_marker.header = goal.header;
    arrow_marker.header.stamp = node_.now();
    arrow_marker.ns = "current_swarm_goal_yaws";
    arrow_marker.id = marker_id++;
    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
    arrow_marker.action = visualization_msgs::msg::Marker::ADD;
    arrow_marker.pose = goal.pose;
    arrow_marker.scale.x = std::max(0.7, perception_.vis_dist);
    arrow_marker.scale.y = 0.1;
    arrow_marker.scale.z = 0.1;
    arrow_marker.color = sphere_marker.color;
    arrow_marker.color.a = 0.9;
    marker_array.markers.push_back(arrow_marker);
  }

  current_goals_marker_pub_->publish(marker_array);
}

void ExplorationVisualizer::clearCurrentGoals()
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = frame_id_;
  clear_marker.header.stamp = node_.now();
  clear_marker.ns = "current_swarm_goals";
  clear_marker.id = 0;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);
  current_goals_marker_pub_->publish(marker_array);
}

void ExplorationVisualizer::publishHGrid(
  const std::vector<GridCellInfo> & grids,
  const std::map<int, std::vector<int>> & assignment)
{
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = frame_id_;
  clear_marker.header.stamp = node_.now();
  clear_marker.ns = "hgrid";
  clear_marker.id = 0;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  int marker_id = 1;
  for (const auto & grid : grids)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = node_.now();
    marker.ns = grid.active ? "hgrid_active" : "hgrid_inactive";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = grid.center.x();
    marker.pose.position.y = grid.center.y();
    marker.pose.position.z = 0.05;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = grid.size;
    marker.scale.y = grid.size;
    marker.scale.z = 0.08;
    marker.color.a = grid.active ? 0.16 : 0.05;
    marker.color.r = grid.active ? 0.1 : 0.4;
    marker.color.g = grid.active ? 0.8 : 0.4;
    marker.color.b = grid.active ? 0.25 : 0.4;
    marker_array.markers.push_back(marker);
  }

  for (const auto & [drone_id, grid_ids] : assignment)
  {
    for (const auto grid_id : grid_ids)
    {
      const auto grid_it = std::find_if(
        grids.begin(), grids.end(),
        [grid_id](const GridCellInfo & grid) {
          return grid.id == grid_id;
        });
      if (grid_it == grids.end())
      {
        continue;
      }

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id_;
      marker.header.stamp = node_.now();
      marker.ns = "hgrid_assignment";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = grid_it->center.x();
      marker.pose.position.y = grid_it->center.y();
      marker.pose.position.z = 0.16 + 0.03 * static_cast<double>(drone_id);
      marker.pose.orientation.w = 1.0;
      marker.scale.x = std::max(0.1, grid_it->size - 0.35);
      marker.scale.y = std::max(0.1, grid_it->size - 0.35);
      marker.scale.z = 0.08;
      marker.color.a = 0.30;
      switch (drone_id)
      {
        case 0:
          marker.color.r = 1.0;
          marker.color.g = 0.0;
          marker.color.b = 0.0;
          break;
        case 1:
          marker.color.r = 0.0;
          marker.color.g = 1.0;
          marker.color.b = 0.0;
          break;
        case 2:
          marker.color.r = 0.0;
          marker.color.g = 0.25;
          marker.color.b = 1.0;
          break;
        case 3:
          marker.color.r = 1.0;
          marker.color.g = 1.0;
          marker.color.b = 0.0;
          break;
        default:
          marker.color.r = 1.0;
          marker.color.g = 1.0;
          marker.color.b = 1.0;
          break;
      }
      marker_array.markers.push_back(marker);
    }
  }

  hgrid_marker_pub_->publish(marker_array);
}

void ExplorationVisualizer::clearHGrid()
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = frame_id_;
  clear_marker.header.stamp = node_.now();
  clear_marker.ns = "hgrid";
  clear_marker.id = 0;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);
  hgrid_marker_pub_->publish(marker_array);
}

void ExplorationVisualizer::publishRoles(const std::vector<DroneState> & swarm_states)
{
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = frame_id_;
  clear_marker.header.stamp = node_.now();
  clear_marker.ns = "swarm_roles";
  clear_marker.id = 0;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  int marker_id = 1;
  for (const auto & state : swarm_states)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = node_.now();
    marker.ns = "swarm_roles";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = state.position.x();
    marker.pose.position.y = state.position.y();
    marker.pose.position.z = state.position.z() + 1.2;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.55;
    marker.color.a = 1.0;
    if (state.role == Role::GARBAGE_COLLECTOR)
    {
      marker.color.r = 1.0;
      marker.color.g = 0.6;
      marker.color.b = 0.0;
    }
    else
    {
      marker.color.r = 0.0;
      marker.color.g = 0.85;
      marker.color.b = 1.0;
    }
    marker.text =
      "drone_" + std::to_string(state.id) + " " + std::string(roleToString(state.role));
    marker_array.markers.push_back(marker);
  }

  role_marker_pub_->publish(marker_array);
}

}  // namespace exploration_swarm
