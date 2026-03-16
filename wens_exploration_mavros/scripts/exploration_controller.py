#!/usr/bin/env python3
"""
exploration_controller.py

High-level exploration controller for WENS UAV autonomous exploration.
Coordinates between MAVROS, the frontier detector, and the waypoint navigator.
"""

import rospy
import math
from enum import Enum

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String


class ExplorationState(Enum):
    IDLE = "IDLE"
    ARMING = "ARMING"
    TAKEOFF = "TAKEOFF"
    EXPLORING = "EXPLORING"
    HOVERING = "HOVERING"
    RETURNING = "RETURNING"
    LANDING = "LANDING"
    DONE = "DONE"
    ERROR = "ERROR"


class ExplorationController:
    def __init__(self):
        rospy.init_node("exploration_controller", anonymous=False)

        # Parameters
        self.takeoff_alt = rospy.get_param("~takeoff_altitude", 2.5)
        self.exploration_radius = rospy.get_param("~exploration_radius", 20.0)
        self.waypoint_tolerance = rospy.get_param("~waypoint_tolerance", 0.5)
        self.hover_duration = rospy.get_param("~hover_duration", 2.0)
        self.max_velocity = rospy.get_param("~max_velocity", 1.5)

        # State
        self.state = ExplorationState.IDLE
        self.current_mavros_state = State()
        self.current_pose = PoseStamped()
        self.frontier_goal = None
        self.last_request = rospy.Time.now()

        # Subscribers
        rospy.Subscriber("mavros/state", State, self._state_cb)
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self._pose_cb)
        rospy.Subscriber("exploration/frontier_goal", PoseStamped, self._frontier_cb)

        # Publishers
        self.setpoint_pub = rospy.Publisher(
            "mavros/setpoint_position/local", PoseStamped, queue_size=10
        )
        self.status_pub = rospy.Publisher(
            "exploration/status", String, queue_size=10
        )

        # Services
        rospy.wait_for_service("mavros/cmd/arming")
        rospy.wait_for_service("mavros/set_mode")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        rospy.loginfo("[ExplorationController] Initialized.")

    def _state_cb(self, msg):
        self.current_mavros_state = msg

    def _pose_cb(self, msg):
        self.current_pose = msg

    def _frontier_cb(self, msg):
        if self.state == ExplorationState.EXPLORING:
            self.frontier_goal = msg
            rospy.loginfo(
                "[ExplorationController] New frontier: (%.2f, %.2f, %.2f)",
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
            )

    def _distance_to(self, pose):
        dx = self.current_pose.pose.position.x - pose.pose.position.x
        dy = self.current_pose.pose.position.y - pose.pose.position.y
        dz = self.current_pose.pose.position.z - pose.pose.position.z
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def _publish_status(self):
        msg = String()
        msg.data = self.state.value
        self.status_pub.publish(msg)

    def _make_setpoint(self, x, y, z):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        return pose

    def run(self):
        rate = rospy.Rate(20)

        # Wait for FCU connection
        rospy.loginfo("[ExplorationController] Waiting for FCU connection...")
        while not rospy.is_shutdown() and not self.current_mavros_state.connected:
            rate.sleep()
        rospy.loginfo("[ExplorationController] FCU connected.")

        # Pre-stream setpoints
        home_setpoint = self._make_setpoint(0.0, 0.0, self.takeoff_alt)
        for _ in range(100):
            self.setpoint_pub.publish(home_setpoint)
            rate.sleep()

        self.state = ExplorationState.ARMING

        while not rospy.is_shutdown():
            self._publish_status()
            now = rospy.Time.now()

            if self.state == ExplorationState.ARMING:
                # Switch to OFFBOARD and arm
                if self.current_mavros_state.mode != "OFFBOARD":
                    if (now - self.last_request).to_sec() > 5.0:
                        resp = self.set_mode_client(custom_mode="OFFBOARD")
                        if resp.mode_sent:
                            rospy.loginfo("[ExplorationController] OFFBOARD mode set.")
                        self.last_request = now
                elif not self.current_mavros_state.armed:
                    if (now - self.last_request).to_sec() > 5.0:
                        resp = self.arming_client(True)
                        if resp.success:
                            rospy.loginfo("[ExplorationController] Vehicle armed.")
                        self.last_request = now
                else:
                    rospy.loginfo("[ExplorationController] Armed and in OFFBOARD. Taking off.")
                    self.state = ExplorationState.TAKEOFF

                self.setpoint_pub.publish(home_setpoint)

            elif self.state == ExplorationState.TAKEOFF:
                self.setpoint_pub.publish(home_setpoint)
                if self._distance_to(home_setpoint) < self.waypoint_tolerance:
                    rospy.loginfo("[ExplorationController] Reached takeoff altitude. Exploring.")
                    self.state = ExplorationState.EXPLORING

            elif self.state == ExplorationState.EXPLORING:
                if self.frontier_goal is not None:
                    self.setpoint_pub.publish(self.frontier_goal)
                    if self._distance_to(self.frontier_goal) < self.waypoint_tolerance:
                        rospy.loginfo("[ExplorationController] Reached frontier. Hovering.")
                        self.hover_start = rospy.Time.now()
                        self.state = ExplorationState.HOVERING
                else:
                    self.setpoint_pub.publish(home_setpoint)

            elif self.state == ExplorationState.HOVERING:
                if self.frontier_goal is not None:
                    self.setpoint_pub.publish(self.frontier_goal)
                if (now - self.hover_start).to_sec() > self.hover_duration:
                    self.frontier_goal = None
                    self.state = ExplorationState.EXPLORING

            elif self.state == ExplorationState.RETURNING:
                self.setpoint_pub.publish(home_setpoint)
                if self._distance_to(home_setpoint) < self.waypoint_tolerance:
                    rospy.loginfo("[ExplorationController] At home. Landing.")
                    self.state = ExplorationState.LANDING

            elif self.state == ExplorationState.LANDING:
                resp = self.set_mode_client(custom_mode="AUTO.LAND")
                if resp.mode_sent:
                    rospy.loginfo("[ExplorationController] Landing initiated.")
                    self.state = ExplorationState.DONE

            elif self.state == ExplorationState.DONE:
                if not hasattr(self, '_done_logged'):
                    rospy.loginfo("[ExplorationController] Mission complete.")
                    self._done_logged = True

            rate.sleep()


if __name__ == "__main__":
    try:
        controller = ExplorationController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
