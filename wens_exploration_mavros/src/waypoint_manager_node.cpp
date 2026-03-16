/**
 * @file waypoint_manager_node.cpp
 * @brief WENS Waypoint Manager – loads and executes a list of waypoints
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <XmlRpcValue.h>
#include <vector>
#include <cmath>
#include <string>

struct Waypoint
{
  double x, y, z;
};

mavros_msgs::State     g_current_state;
geometry_msgs::PoseStamped g_current_pose;

void stateCb(const mavros_msgs::State::ConstPtr& msg)  { g_current_state = *msg; }
void poseCb (const geometry_msgs::PoseStamped::ConstPtr& msg) { g_current_pose  = *msg; }

double dist3d(const geometry_msgs::PoseStamped& pose, const Waypoint& wp)
{
  double dx = pose.pose.position.x - wp.x;
  double dy = pose.pose.position.y - wp.y;
  double dz = pose.pose.position.z - wp.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wens_waypoint_manager_node");
  ros::NodeHandle nh("~");

  double waypoint_tolerance, hover_time, default_altitude;
  nh.param("waypoint_tolerance", waypoint_tolerance, 0.5);
  nh.param("hover_time",         hover_time,         2.0);
  nh.param("default_altitude",   default_altitude,   3.0);

  // Load waypoints from parameter server
  std::vector<Waypoint> waypoints;
  XmlRpc::XmlRpcValue wp_list;
  if (nh.getParam("waypoints", wp_list) && wp_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (int i = 0; i < wp_list.size(); ++i)
    {
      Waypoint wp;
      wp.x = wp_list[i].hasMember("x") ? static_cast<double>(wp_list[i]["x"]) : 0.0;
      wp.y = wp_list[i].hasMember("y") ? static_cast<double>(wp_list[i]["y"]) : 0.0;
      wp.z = wp_list[i].hasMember("z") ? static_cast<double>(wp_list[i]["z"]) : default_altitude;
      waypoints.push_back(wp);
    }
  }

  if (waypoints.empty())
  {
    ROS_WARN("No waypoints loaded. Exiting.");
    return 0;
  }

  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
    "/mavros/setpoint_position/local", 10);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
    "/mavros/state", 10, stateCb);
  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
    "/mavros/local_position/pose", 10, poseCb);

  ros::ServiceClient arming_client   = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

  ros::service::waitForService("/mavros/cmd/arming");
  ros::service::waitForService("/mavros/set_mode");

  ros::Rate rate(20.0);

  // Pre-populate setpoint stream
  geometry_msgs::PoseStamped first_wp_pose;
  first_wp_pose.pose.position.x  = waypoints[0].x;
  first_wp_pose.pose.position.y  = waypoints[0].y;
  first_wp_pose.pose.position.z  = waypoints[0].z;
  first_wp_pose.pose.orientation.w = 1.0;

  for (int i = 0; i < 100 && ros::ok(); ++i)
  {
    first_wp_pose.header.stamp = ros::Time::now();
    local_pos_pub.publish(first_wp_pose);
    ros::spinOnce();
    rate.sleep();
  }

  // Arm and OFFBOARD
  mavros_msgs::SetMode offboard_mode;
  offboard_mode.request.custom_mode = "OFFBOARD";
  set_mode_client.call(offboard_mode);

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  arming_client.call(arm_cmd);

  size_t wp_index = 0;
  ROS_INFO("Starting waypoint following (%zu waypoints).", waypoints.size());

  while (ros::ok())
  {
    if (wp_index >= waypoints.size())
    {
      ROS_INFO("All waypoints reached. Landing.");
      mavros_msgs::SetMode land_mode;
      land_mode.request.custom_mode = "AUTO.LAND";
      set_mode_client.call(land_mode);
      break;
    }

    const Waypoint& wp = waypoints[wp_index];
    geometry_msgs::PoseStamped setpoint;
    setpoint.header.stamp       = ros::Time::now();
    setpoint.pose.position.x    = wp.x;
    setpoint.pose.position.y    = wp.y;
    setpoint.pose.position.z    = wp.z;
    setpoint.pose.orientation.w = 1.0;
    local_pos_pub.publish(setpoint);

    if (dist3d(g_current_pose, wp) < waypoint_tolerance)
    {
      ROS_INFO("Reached waypoint %zu / %zu.", wp_index + 1, waypoints.size());
      ros::Duration(hover_time).sleep();
      ++wp_index;
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
