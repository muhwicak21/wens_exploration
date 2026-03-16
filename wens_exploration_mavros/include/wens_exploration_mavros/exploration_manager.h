/**
 * @file exploration_manager.h
 * @brief Header for ExplorationManager class
 */

#ifndef WENS_EXPLORATION_MAVROS_EXPLORATION_MANAGER_H
#define WENS_EXPLORATION_MAVROS_EXPLORATION_MANAGER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <std_msgs/String.h>

namespace wens_exploration {

enum class ExplorationState {
    INIT,
    TAKEOFF,
    EXPLORING,
    RETURNING,
    LANDING,
    DONE
};

std::string stateToString(ExplorationState state);

}  // namespace wens_exploration

#endif  // WENS_EXPLORATION_MAVROS_EXPLORATION_MANAGER_H
