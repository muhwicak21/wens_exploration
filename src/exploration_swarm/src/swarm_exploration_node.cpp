#include <memory>

#include "exploration_swarm/exploration_goal_manager.hpp"
#include "rclcpp/rclcpp.hpp"

namespace exploration_swarm
{

class SwarmExplorationNode : public rclcpp::Node
{
public:
  SwarmExplorationNode() : Node("swarm_exploration_node"), goal_manager_(*this) {}

private:
  ExplorationGoalManager goal_manager_;
};

}  // namespace exploration_swarm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<exploration_swarm::SwarmExplorationNode>());
  rclcpp::shutdown();
  return 0;
}
