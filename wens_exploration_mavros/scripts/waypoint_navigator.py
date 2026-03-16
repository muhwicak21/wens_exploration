#!/usr/bin/env python3
"""
waypoint_navigator.py

Waypoint-based navigation node for WENS UAV exploration.
Accepts a sequence of waypoints and navigates the UAV through them
using MAVROS setpoint_position/local.
"""

import rospy
import math
import yaml
import os
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import Bool, String


class WaypointNavigator:
    def __init__(self):
        rospy.init_node("waypoint_navigator", anonymous=False)

        # Parameters
        self.waypoint_tolerance = rospy.get_param("~waypoint_tolerance", 0.5)
        self.waypoint_file = rospy.get_param("~waypoint_file", "")
        self.loop_mission = rospy.get_param("~loop_mission", False)
        self.default_altitude = rospy.get_param("~default_altitude", 2.5)

        # State
        self.waypoints = []
        self.current_wp_idx = 0
        self.current_pose = PoseStamped()
        self.mavros_state = State()
        self.mission_active = False

        # Load waypoints from file if provided
        if self.waypoint_file:
            self._load_waypoints(self.waypoint_file)

        # Subscribers
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self._pose_cb)
        rospy.Subscriber("mavros/state", State, self._state_cb)
        rospy.Subscriber("exploration/mission_start", Bool, self._mission_start_cb)

        # Publishers
        self.setpoint_pub = rospy.Publisher(
            "mavros/setpoint_position/local", PoseStamped, queue_size=10
        )
        self.mission_status_pub = rospy.Publisher(
            "exploration/waypoint_status", String, queue_size=10
        )
        self.mission_done_pub = rospy.Publisher(
            "exploration/mission_done", Bool, queue_size=10
        )

        rospy.loginfo("[WaypointNavigator] Initialized with %d waypoints.", len(self.waypoints))

    def _load_waypoints(self, filepath):
        """Load waypoints from a YAML file."""
        try:
            with open(filepath, "r") as f:
                data = yaml.safe_load(f)
            if "waypoints" in data:
                for wp in data["waypoints"]:
                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.pose.position.x = wp.get("x", 0.0)
                    pose.pose.position.y = wp.get("y", 0.0)
                    pose.pose.position.z = wp.get("z", self.default_altitude)
                    pose.pose.orientation.w = 1.0
                    self.waypoints.append(pose)
                rospy.loginfo("[WaypointNavigator] Loaded %d waypoints from %s",
                              len(self.waypoints), filepath)
        except Exception as e:
            rospy.logerr("[WaypointNavigator] Failed to load waypoints: %s", str(e))

    def add_waypoint(self, x, y, z=None):
        """Add a waypoint programmatically."""
        if z is None:
            z = self.default_altitude
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        self.waypoints.append(pose)

    def _pose_cb(self, msg):
        self.current_pose = msg

    def _state_cb(self, msg):
        self.mavros_state = msg

    def _mission_start_cb(self, msg):
        if msg.data:
            self.mission_active = True
            self.current_wp_idx = 0
            rospy.loginfo("[WaypointNavigator] Mission started.")
        else:
            self.mission_active = False
            rospy.loginfo("[WaypointNavigator] Mission paused.")

    def _distance_to(self, target_pose):
        dx = self.current_pose.pose.position.x - target_pose.pose.position.x
        dy = self.current_pose.pose.position.y - target_pose.pose.position.y
        dz = self.current_pose.pose.position.z - target_pose.pose.position.z
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def run(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if self.mission_active and self.waypoints:
                if self.current_wp_idx < len(self.waypoints):
                    current_wp = self.waypoints[self.current_wp_idx]
                    current_wp.header.stamp = rospy.Time.now()
                    self.setpoint_pub.publish(current_wp)

                    # Publish status
                    status_msg = String()
                    status_msg.data = "WP_{}/{}".format(
                        self.current_wp_idx + 1, len(self.waypoints)
                    )
                    self.mission_status_pub.publish(status_msg)

                    if self._distance_to(current_wp) < self.waypoint_tolerance:
                        rospy.loginfo(
                            "[WaypointNavigator] Reached waypoint %d/%d",
                            self.current_wp_idx + 1, len(self.waypoints)
                        )
                        self.current_wp_idx += 1

                else:
                    # Mission complete
                    if self.loop_mission:
                        self.current_wp_idx = 0
                        rospy.loginfo("[WaypointNavigator] Looping mission.")
                    else:
                        self.mission_active = False
                        done_msg = Bool()
                        done_msg.data = True
                        self.mission_done_pub.publish(done_msg)
                        rospy.loginfo("[WaypointNavigator] All waypoints reached. Mission done.")

            rate.sleep()


if __name__ == "__main__":
    try:
        navigator = WaypointNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
