/**
 * @file exploration_node.cpp
 * @brief WENS Exploration node – C++ implementation
 *
 * Implements a coverage-path exploration strategy for MAVLink drones via MAVROS.
 */

#include <wens_exploration_mavros/exploration_node.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

namespace wens_exploration_mavros
{

ExplorationNode::ExplorationNode(ros::NodeHandle& nh)
  : nh_(nh), current_wp_index_(0)
{
  nh_.param("takeoff_altitude",    takeoff_altitude_,    3.0);
  nh_.param("exploration_altitude", exploration_altitude_, 5.0);
  nh_.param("grid_spacing",        grid_spacing_,        5.0);
  nh_.param("grid_rows",           grid_rows_,           4);
  nh_.param("grid_cols",           grid_cols_,           4);
  nh_.param("waypoint_tolerance",  waypoint_tolerance_,  0.5);
  nh_.param("hover_time",          hover_time_,          2.0);

  local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
    "mavros/setpoint_position/local", 10);

  state_sub_ = nh_.subscribe<mavros_msgs::State>(
    "mavros/state", 10, &ExplorationNode::stateCb, this);
  pose_sub_  = nh_.subscribe<geometry_msgs::PoseStamped>(
    "mavros/local_position/pose", 10, &ExplorationNode::poseCb, this);

  arming_client_   = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  ros::service::waitForService("mavros/cmd/arming");
  ros::service::waitForService("mavros/set_mode");

  generateCoveragePath();

  ROS_INFO("WENS ExplorationNode initialized with %zu waypoints.", waypoints_.size());
}

void ExplorationNode::stateCb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state_ = *msg;
}

void ExplorationNode::poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_pose_ = *msg;
}

void ExplorationNode::generateCoveragePath()
{
  waypoints_.clear();
  for (int row = 0; row < grid_rows_; ++row)
  {
    int col_start = (row % 2 == 0) ? 0 : grid_cols_ - 1;
    int col_end   = (row % 2 == 0) ? grid_cols_ : -1;
    int col_step  = (row % 2 == 0) ? 1 : -1;

    for (int col = col_start; col != col_end; col += col_step)
    {
      Waypoint wp;
      wp.x = col * grid_spacing_;
      wp.y = row * grid_spacing_;
      wp.z = exploration_altitude_;
      waypoints_.push_back(wp);
    }
  }
}

double ExplorationNode::distanceToWaypoint(const Waypoint& wp) const
{
  double dx = current_pose_.pose.position.x - wp.x;
  double dy = current_pose_.pose.position.y - wp.y;
  double dz = current_pose_.pose.position.z - wp.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool ExplorationNode::setOffboardMode()
{
  mavros_msgs::SetMode mode_req;
  mode_req.request.custom_mode = "OFFBOARD";
  if (set_mode_client_.call(mode_req) && mode_req.response.mode_sent)
  {
    ROS_INFO("OFFBOARD mode enabled.");
    return true;
  }
  ROS_WARN("Failed to set OFFBOARD mode.");
  return false;
}

bool ExplorationNode::armVehicle()
{
  mavros_msgs::CommandBool arm_req;
  arm_req.request.value = true;
  if (arming_client_.call(arm_req) && arm_req.response.success)
  {
    ROS_INFO("Vehicle armed.");
    return true;
  }
  ROS_WARN("Failed to arm vehicle.");
  return false;
}

void ExplorationNode::returnToLaunch()
{
  mavros_msgs::SetMode mode_req;
  mode_req.request.custom_mode = "AUTO.RTL";
  if (set_mode_client_.call(mode_req) && mode_req.response.mode_sent)
  {
    ROS_INFO("RTL mode enabled.");
  }
}

void ExplorationNode::run()
{
  ros::Rate rate(20.0);

  // Pre-populate setpoint stream before switching to OFFBOARD
  geometry_msgs::PoseStamped takeoff_pose;
  takeoff_pose.pose.position.z  = takeoff_altitude_;
  takeoff_pose.pose.orientation.w = 1.0;

  for (int i = 0; i < 100 && ros::ok(); ++i)
  {
    takeoff_pose.header.stamp = ros::Time::now();
    local_pos_pub_.publish(takeoff_pose);
    ros::spinOnce();
    rate.sleep();
  }

  setOffboardMode();
  armVehicle();

  ROS_INFO("Starting WENS exploration...");

  while (ros::ok())
  {
    if (current_wp_index_ >= waypoints_.size())
    {
      ROS_INFO("Exploration complete. Returning to launch.");
      returnToLaunch();
      break;
    }

    const Waypoint& wp = waypoints_[current_wp_index_];

    geometry_msgs::PoseStamped setpoint;
    setpoint.header.stamp     = ros::Time::now();
    setpoint.pose.position.x  = wp.x;
    setpoint.pose.position.y  = wp.y;
    setpoint.pose.position.z  = wp.z;
    setpoint.pose.orientation.w = 1.0;
    local_pos_pub_.publish(setpoint);

    if (distanceToWaypoint(wp) < waypoint_tolerance_)
    {
      ROS_INFO("Reached waypoint %zu / %zu.", current_wp_index_ + 1, waypoints_.size());
      ros::Duration(hover_time_).sleep();
      ++current_wp_index_;
    }

    ros::spinOnce();
    rate.sleep();
  }
}

}  // namespace wens_exploration_mavros

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wens_exploration_node");
  ros::NodeHandle nh("~");
  wens_exploration_mavros::ExplorationNode node(nh);
  node.run();
  return 0;
}
