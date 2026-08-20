#include <algorithm>
#include <chrono>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class ExplorationGoalDummy : public rclcpp::Node
{
public:
  ExplorationGoalDummy() : Node("exploration_goal_dummy"), rng_(std::random_device{}())
  {
    declare_parameter("publish_period", 5.0);
    declare_parameter("frame_id", "world");
    declare_parameter("map_size_x", 42.0);
    declare_parameter("map_size_y", 30.0);
    declare_parameter("map_size_z", 5.0);
    declare_parameter("map_margin", 1.0);
    declare_parameter("min_z", 0.8);

    get_parameter("publish_period", publish_period_);
    get_parameter("frame_id", frame_id_);
    get_parameter("map_size_x", map_size_x_);
    get_parameter("map_size_y", map_size_y_);
    get_parameter("map_size_z", map_size_z_);
    get_parameter("map_margin", map_margin_);
    get_parameter("min_z", min_z_);

    for (size_t i = 0; i < drone_num_; ++i)
    {
      const std::string topic = "/drone_" + std::to_string(i) + "_planning/exploration_goal";
      pubs_.push_back(create_publisher<geometry_msgs::msg::PoseStamped>(topic, 1));
      RCLCPP_INFO(get_logger(), "Publishing random exploration goals for drone %zu on %s", i, topic.c_str());
    }

    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(publish_period_)),
        std::bind(&ExplorationGoalDummy::publishGoals, this));
  }

private:
  struct Goal
  {
    double x;
    double y;
    double z;
  };

  Goal randomGoal()
  {
    const double max_x = std::max(0.0, map_size_x_ * 0.5 - map_margin_);
    const double max_y = std::max(0.0, map_size_y_ * 0.5 - map_margin_);
    const double max_z = std::max(min_z_, map_size_z_ - map_margin_);

    std::uniform_real_distribution<double> x_dist(-max_x, max_x);
    std::uniform_real_distribution<double> y_dist(-max_y, max_y);
    std::uniform_real_distribution<double> z_dist(min_z_, max_z);

    return {x_dist(rng_), y_dist(rng_), z_dist(rng_)};
  }

  void publishGoals()
  {
    for (size_t i = 0; i < pubs_.size(); ++i)
    {
      const Goal random_goal = randomGoal();

      geometry_msgs::msg::PoseStamped goal;
      goal.header.stamp = now();
      goal.header.frame_id = frame_id_;
      goal.pose.position.x = random_goal.x;
      goal.pose.position.y = random_goal.y;
      goal.pose.position.z = random_goal.z;
      goal.pose.orientation.w = 1.0;

      pubs_[i]->publish(goal);

      RCLCPP_INFO(get_logger(), "Drone %zu random exploration goal: [%.2f, %.2f, %.2f]",
                  i, random_goal.x, random_goal.y, random_goal.z);
    }
  }

  static constexpr size_t drone_num_ = 4;
  double publish_period_;
  double map_size_x_;
  double map_size_y_;
  double map_size_z_;
  double map_margin_;
  double min_z_;
  std::string frame_id_;
  std::mt19937 rng_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pubs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExplorationGoalDummy>());
  rclcpp::shutdown();
  return 0;
}
