/**
 * @file exploration_manager.cpp
 * @brief Main exploration manager for WENS UAV autonomous exploration using MAVROS.
 *
 * This node coordinates the overall exploration mission:
 * - Arms the vehicle and switches to OFFBOARD mode
 * - Takes off to a specified altitude
 * - Requests frontier goals from frontier_detector
 * - Sends waypoints to waypoint_navigator
 * - Monitors mission completion and landing
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

class ExplorationManager {
public:
    ExplorationManager() {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        // Parameters
        pnh.param("takeoff_altitude", takeoff_altitude_, 2.0);
        pnh.param("exploration_timeout", exploration_timeout_, 300.0);

        // Subscribers
        state_sub_ = nh.subscribe<mavros_msgs::State>(
            "mavros/state", 10, &ExplorationManager::stateCallback, this);
        local_pos_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
            "mavros/local_position/pose", 10, &ExplorationManager::localPosCallback, this);
        frontier_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
            "exploration/frontier_goal", 10, &ExplorationManager::frontierCallback, this);

        // Publishers
        local_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
            "mavros/setpoint_position/local", 10);
        exploration_status_pub_ = nh.advertise<std_msgs::String>(
            "exploration/status", 10);

        // Service clients
        arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>(
            "mavros/cmd/arming");
        set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>(
            "mavros/set_mode");

        // Initialize state
        current_state_ = ExplorationState::INIT;
        mission_start_time_ = ros::Time::now();
        has_frontier_ = false;

        ROS_INFO("[ExplorationManager] Initialized. Takeoff altitude: %.1f m", takeoff_altitude_);
    }

    void run() {
        ros::Rate rate(20.0);

        // Wait for FCU connection
        ROS_INFO("[ExplorationManager] Waiting for FCU connection...");
        while (ros::ok() && !current_mavros_state_.connected) {
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("[ExplorationManager] FCU connected.");

        // Pre-send setpoints for mode switching
        geometry_msgs::PoseStamped takeoff_pose;
        takeoff_pose.pose.position.x = 0.0;
        takeoff_pose.pose.position.y = 0.0;
        takeoff_pose.pose.position.z = takeoff_altitude_;
        for (int i = 100; ros::ok() && i > 0; --i) {
            local_pos_pub_.publish(takeoff_pose);
            ros::spinOnce();
            rate.sleep();
        }

        mavros_msgs::SetMode offboard_mode;
        offboard_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        ros::Time last_request_time = ros::Time::now();

        while (ros::ok()) {
            // Arm and switch to OFFBOARD
            if (current_mavros_state_.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request_time > ros::Duration(5.0))) {
                if (set_mode_client_.call(offboard_mode) &&
                    offboard_mode.response.mode_sent) {
                    ROS_INFO("[ExplorationManager] OFFBOARD mode enabled.");
                }
                last_request_time = ros::Time::now();
            } else if (!current_mavros_state_.armed &&
                       (ros::Time::now() - last_request_time > ros::Duration(5.0))) {
                if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("[ExplorationManager] Vehicle armed.");
                }
                last_request_time = ros::Time::now();
            }

            // State machine
            switch (current_state_) {
                case ExplorationState::INIT:
                    publishStatus("INIT");
                    if (current_mavros_state_.armed &&
                        current_mavros_state_.mode == "OFFBOARD") {
                        current_state_ = ExplorationState::TAKEOFF;
                        target_pose_ = takeoff_pose;
                        ROS_INFO("[ExplorationManager] Transitioning to TAKEOFF.");
                    }
                    local_pos_pub_.publish(takeoff_pose);
                    break;

                case ExplorationState::TAKEOFF:
                    publishStatus("TAKEOFF");
                    local_pos_pub_.publish(target_pose_);
                    if (isAtTarget(0.0, 0.0, takeoff_altitude_)) {
                        current_state_ = ExplorationState::EXPLORING;
                        mission_start_time_ = ros::Time::now();
                        ROS_INFO("[ExplorationManager] Takeoff complete. Starting exploration.");
                    }
                    break;

                case ExplorationState::EXPLORING:
                    publishStatus("EXPLORING");
                    if (has_frontier_) {
                        local_pos_pub_.publish(frontier_goal_);
                        has_frontier_ = false;
                    } else {
                        local_pos_pub_.publish(target_pose_);
                    }
                    if ((ros::Time::now() - mission_start_time_).toSec() > exploration_timeout_) {
                        current_state_ = ExplorationState::RETURNING;
                        ROS_INFO("[ExplorationManager] Exploration timeout. Returning to home.");
                    }
                    break;

                case ExplorationState::RETURNING: {
                    publishStatus("RETURNING");
                    geometry_msgs::PoseStamped home_pose;
                    home_pose.header.stamp = ros::Time::now();
                    home_pose.header.frame_id = "map";
                    home_pose.pose.position.x = 0.0;
                    home_pose.pose.position.y = 0.0;
                    home_pose.pose.position.z = takeoff_altitude_;
                    local_pos_pub_.publish(home_pose);
                    if (isAtTarget(0.0, 0.0, takeoff_altitude_)) {
                        current_state_ = ExplorationState::LANDING;
                        ROS_INFO("[ExplorationManager] At home position. Landing.");
                    }
                    break;
                }

                case ExplorationState::LANDING: {
                    publishStatus("LANDING");
                    mavros_msgs::SetMode land_mode;
                    land_mode.request.custom_mode = "AUTO.LAND";
                    if (set_mode_client_.call(land_mode) && land_mode.response.mode_sent) {
                        ROS_INFO("[ExplorationManager] Landing initiated.");
                        current_state_ = ExplorationState::DONE;
                    }
                    break;
                }

                case ExplorationState::DONE:
                    publishStatus("DONE");
                    ROS_INFO_ONCE("[ExplorationManager] Mission complete.");
                    break;
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    enum class ExplorationState {
        INIT, TAKEOFF, EXPLORING, RETURNING, LANDING, DONE
    };

    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        current_mavros_state_ = *msg;
    }

    void localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
    }

    void frontierCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (current_state_ == ExplorationState::EXPLORING) {
            frontier_goal_ = *msg;
            target_pose_ = *msg;
            has_frontier_ = true;
            ROS_INFO("[ExplorationManager] New frontier goal: (%.2f, %.2f, %.2f)",
                     msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        }
    }

    bool isAtTarget(double tx, double ty, double tz) {
        double cx = current_pose_.pose.position.x;
        double cy = current_pose_.pose.position.y;
        double cz = current_pose_.pose.position.z;
        double dist = std::sqrt(std::pow(cx - tx, 2) + std::pow(cy - ty, 2) + std::pow(cz - tz, 2));
        return dist < 0.3;
    }

    bool isAtTargetPose(const geometry_msgs::PoseStamped& target) {
        return isAtTarget(
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z);
    }

    void publishStatus(const std::string& status) {
        std_msgs::String msg;
        msg.data = status;
        exploration_status_pub_.publish(msg);
    }

    // ROS handles
    ros::Subscriber state_sub_, local_pos_sub_, frontier_sub_;
    ros::Publisher local_pos_pub_, exploration_status_pub_;
    ros::ServiceClient arming_client_, set_mode_client_;

    // State
    ExplorationState current_state_;
    mavros_msgs::State current_mavros_state_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped target_pose_;
    geometry_msgs::PoseStamped frontier_goal_;
    bool has_frontier_;
    ros::Time mission_start_time_;

    // Parameters
    double takeoff_altitude_;
    double exploration_timeout_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "exploration_manager");
    ExplorationManager manager;
    manager.run();
    return 0;
}
