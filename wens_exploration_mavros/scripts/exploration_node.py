#!/usr/bin/env python3
"""
WENS Exploration Node
Autonomous exploration for MAVLink-based drones via MAVROS.

This node implements a frontier-based exploration strategy:
1. Arms the vehicle and switches to OFFBOARD mode
2. Takes off to a configured altitude
3. Explores the environment using a coverage path
4. Returns to launch when battery is low or area is covered
"""

import rospy
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from std_msgs.msg import Float64


class WENSExplorationNode:
    """Waypoint Exploration Navigation System node."""

    def __init__(self):
        rospy.init_node('wens_exploration_node', anonymous=True)

        # Parameters
        self.takeoff_altitude = rospy.get_param('~takeoff_altitude', 3.0)
        self.exploration_altitude = rospy.get_param('~exploration_altitude', 5.0)
        self.grid_spacing = rospy.get_param('~grid_spacing', 5.0)
        self.grid_rows = rospy.get_param('~grid_rows', 4)
        self.grid_cols = rospy.get_param('~grid_cols', 4)
        self.waypoint_tolerance = rospy.get_param('~waypoint_tolerance', 0.5)
        self.hover_time = rospy.get_param('~hover_time', 2.0)
        self.battery_return_threshold = rospy.get_param('~battery_return_threshold', 20.0)

        # State
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.battery_level = 100.0
        self.waypoints = []
        self.current_wp_index = 0

        # Publishers
        self.local_pos_pub = rospy.Publisher(
            'mavros/setpoint_position/local',
            PoseStamped,
            queue_size=10
        )

        # Subscribers
        rospy.Subscriber('mavros/state', State, self.state_callback)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('mavros/battery', Float64, self.battery_callback)

        # Service clients
        rospy.wait_for_service('mavros/cmd/arming')
        rospy.wait_for_service('mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

        # Generate coverage waypoints
        self.generate_coverage_path()

        rospy.loginfo("WENS Exploration Node initialized.")

    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_pose = msg

    def battery_callback(self, msg):
        self.battery_level = msg.data

    def generate_coverage_path(self):
        """Generate a boustrophedon (lawnmower) coverage path."""
        self.waypoints = []
        for row in range(self.grid_rows):
            cols = range(self.grid_cols) if row % 2 == 0 else range(self.grid_cols - 1, -1, -1)
            for col in cols:
                wp = PoseStamped()
                wp.pose.position.x = col * self.grid_spacing
                wp.pose.position.y = row * self.grid_spacing
                wp.pose.position.z = self.exploration_altitude
                wp.pose.orientation.w = 1.0
                self.waypoints.append(wp)
        rospy.loginfo("Generated %d coverage waypoints.", len(self.waypoints))

    def distance_to_waypoint(self, wp):
        """Compute 3D Euclidean distance to a waypoint."""
        dx = self.current_pose.pose.position.x - wp.pose.position.x
        dy = self.current_pose.pose.position.y - wp.pose.position.y
        dz = self.current_pose.pose.position.z - wp.pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def set_offboard_mode(self):
        """Switch to OFFBOARD flight mode."""
        mode_req = SetModeRequest()
        mode_req.custom_mode = 'OFFBOARD'
        if self.set_mode_client.call(mode_req).mode_sent:
            rospy.loginfo("OFFBOARD mode enabled.")
            return True
        rospy.logwarn("Failed to set OFFBOARD mode.")
        return False

    def arm_vehicle(self):
        """Arm the vehicle."""
        arm_req = CommandBoolRequest()
        arm_req.value = True
        if self.arming_client.call(arm_req).success:
            rospy.loginfo("Vehicle armed.")
            return True
        rospy.logwarn("Failed to arm vehicle.")
        return False

    def return_to_launch(self):
        """Command a Return-to-Launch."""
        mode_req = SetModeRequest()
        mode_req.custom_mode = 'AUTO.RTL'
        if self.set_mode_client.call(mode_req).mode_sent:
            rospy.loginfo("RTL mode enabled.")

    def run(self):
        rate = rospy.Rate(20)

        # Pre-populate setpoint stream before switching to OFFBOARD
        takeoff_pose = PoseStamped()
        takeoff_pose.pose.position.z = self.takeoff_altitude
        takeoff_pose.pose.orientation.w = 1.0

        for _ in range(100):
            takeoff_pose.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(takeoff_pose)
            rate.sleep()

        # Arm and switch to OFFBOARD
        self.set_offboard_mode()
        self.arm_vehicle()

        rospy.loginfo("Starting WENS exploration...")

        while not rospy.is_shutdown():
            if self.battery_level < self.battery_return_threshold:
                rospy.logwarn("Low battery (%.1f%%). Returning to launch.", self.battery_level)
                self.return_to_launch()
                break

            if self.current_wp_index >= len(self.waypoints):
                rospy.loginfo("Exploration complete. Returning to launch.")
                self.return_to_launch()
                break

            wp = self.waypoints[self.current_wp_index]
            wp.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(wp)

            if self.distance_to_waypoint(wp) < self.waypoint_tolerance:
                rospy.loginfo("Reached waypoint %d / %d.",
                              self.current_wp_index + 1, len(self.waypoints))
                rospy.sleep(self.hover_time)
                self.current_wp_index += 1

            rate.sleep()


if __name__ == '__main__':
    try:
        node = WENSExplorationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
