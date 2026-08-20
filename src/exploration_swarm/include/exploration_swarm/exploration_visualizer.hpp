#pragma once

#include <string>
#include <map>
#include <vector>

#include "exploration_swarm/exploration_common.hpp"
#include "exploration_swarm/hgrid_allocator.hpp"
#include "exploration_swarm/role_assigner.hpp"
#include "exploration_swarm/viewpoint_sampler.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace exploration_swarm
{

class ExplorationVisualizer
{
public:
  explicit ExplorationVisualizer(rclcpp::Node & node);

  void setFrameId(const std::string & frame_id);
  void setPerceptionParameters(const PerceptionParameters & perception);
  void publishFrontiers(const std::vector<FrontierCluster> & clusters);
  void publishViewpoints(const std::vector<Viewpoint> & viewpoints);
  void publishCurrentGoal(const geometry_msgs::msg::PoseStamped & goal);
  void publishCurrentGoals(const std::vector<geometry_msgs::msg::PoseStamped> & goals);
  void publishHGrid(
    const std::vector<GridCellInfo> & grids,
    const std::map<int, std::vector<int>> & assignment);
  void clearHGrid();
  void publishRoles(const std::vector<DroneState> & swarm_states);
  void clearCurrentGoal();
  void clearCurrentGoals();

private:
  rclcpp::Node & node_;
  std::string frame_id_{"world"};
  PerceptionParameters perception_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viewpoint_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr current_goal_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr current_goals_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hgrid_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr role_marker_pub_;
};

}  // namespace exploration_swarm
