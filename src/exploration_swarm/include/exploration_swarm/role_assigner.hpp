#pragma once

#include <cstddef>
#include <vector>

#include "Eigen/Core"
#include "exploration_swarm/exploration_common.hpp"

namespace exploration_swarm
{

enum class Role : size_t
{
  EXPLORER = 0,
  GARBAGE_COLLECTOR = 1,
  UNKNOWN = 2
};

struct DroneState
{
  int id{0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d current_goal{Eigen::Vector3d::Zero()};
  bool has_goal{false};
  Role role{Role::UNKNOWN};
};

class RoleAssigner
{
public:
  RoleAssigner() = default;

  void setParameters(const RoleParams & params);

  Role assignRole(
    int drone_id,
    const Eigen::Vector3d & position,
    const Eigen::Vector3d & velocity,
    const std::vector<DroneState> & swarm_states,
    const std::vector<FrontierCluster> & frontiers);

private:
  Role assignRoleFromNearbyFrontiers(
    int drone_id,
    const Eigen::Vector3d & position,
    const std::vector<DroneState> & swarm_states,
    const std::vector<FrontierCluster> & frontiers);

  RoleParams params_;
};

const char * roleToString(Role role);

}  // namespace exploration_swarm
