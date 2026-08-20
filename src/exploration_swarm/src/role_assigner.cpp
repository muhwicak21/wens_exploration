#include "exploration_swarm/role_assigner.hpp"

#include <algorithm>
#include <cmath>

namespace exploration_swarm
{

void RoleAssigner::setParameters(const RoleParams & params)
{
  params_ = params;
  params_.region_size = std::max(0.1, params_.region_size);
  params_.cluster_xy_size = std::max(0.1, params_.cluster_xy_size);
  params_.min_num_neighbours = std::max(0, params_.min_num_neighbours);
  params_.min_num_islands = std::max(0, params_.min_num_islands);
  params_.min_num_trails = std::max(0, params_.min_num_trails);
}

Role RoleAssigner::assignRole(
  const int drone_id,
  const Eigen::Vector3d & position,
  const Eigen::Vector3d & velocity,
  const std::vector<DroneState> & swarm_states,
  const std::vector<FrontierCluster> & frontiers)
{
  (void)velocity;
  if (params_.fixed)
  {
    return params_.fix_role == 1 ? Role::GARBAGE_COLLECTOR : Role::EXPLORER;
  }

  return assignRoleFromNearbyFrontiers(drone_id, position, swarm_states, frontiers);
}

Role RoleAssigner::assignRoleFromNearbyFrontiers(
  const int drone_id,
  const Eigen::Vector3d & position,
  const std::vector<DroneState> & swarm_states,
  const std::vector<FrontierCluster> & frontiers)
{
  int nearby_frontiers = 0;
  int nearby_islands = 0;
  int nearby_trails = 0;

  for (const auto & frontier : frontiers)
  {
    const double distance = (frontier.average - position).head<2>().norm();
    if (distance > params_.region_size)
    {
      continue;
    }

    ++nearby_frontiers;
    const double xy_size =
      std::max(
      frontier.box_max.x() - frontier.box_min.x(),
      frontier.box_max.y() - frontier.box_min.y());
    if (xy_size <= params_.cluster_xy_size)
    {
      ++nearby_islands;
    }
    if (frontier.label == FrontierLabel::TRAIL)
    {
      ++nearby_trails;
    }
  }

  bool nearby_collector = false;
  for (const auto & state : swarm_states)
  {
    if (state.id == drone_id || state.role != Role::GARBAGE_COLLECTOR)
    {
      continue;
    }
    if ((state.position - position).head<2>().norm() <= params_.region_size)
    {
      nearby_collector = true;
      break;
    }
  }

  if (nearby_collector)
  {
    return Role::EXPLORER;
  }

  if (nearby_frontiers >= params_.min_num_neighbours &&
    nearby_islands < params_.min_num_islands &&
    nearby_trails <= params_.min_num_trails)
  {
    return Role::EXPLORER;
  }

  return Role::GARBAGE_COLLECTOR;
}

const char * roleToString(const Role role)
{
  switch (role)
  {
    case Role::EXPLORER:
      return "EXPLORER";
    case Role::GARBAGE_COLLECTOR:
      return "GARBAGE_COLLECTOR";
    case Role::UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

}  // namespace exploration_swarm
