#pragma once
/**
 * @file exploration_node.h
 * @brief WENS Exploration node C++ header
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <vector>
#include <cmath>

namespace wens_exploration_mavros
{

struct Waypoint
{
  double x;
  double y;
  double z;
};

class ExplorationNode
{
public:
  explicit ExplorationNode(ros::NodeHandle& nh);
  void run();

private:
  void stateCb(const mavros_msgs::State::ConstPtr& msg);
  void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void generateCoveragePath();
  double distanceToWaypoint(const Waypoint& wp) const;
  bool setOffboardMode();
  bool armVehicle();
  void returnToLaunch();

  ros::NodeHandle nh_;
  ros::Publisher  local_pos_pub_;
  ros::Subscriber state_sub_;
  ros::Subscriber pose_sub_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;

  mavros_msgs::State     current_state_;
  geometry_msgs::PoseStamped current_pose_;
  std::vector<Waypoint>  waypoints_;
  size_t current_wp_index_;

  double takeoff_altitude_;
  double exploration_altitude_;
  double grid_spacing_;
  int    grid_rows_;
  int    grid_cols_;
  double waypoint_tolerance_;
  double hover_time_;
};

}  // namespace wens_exploration_mavros
