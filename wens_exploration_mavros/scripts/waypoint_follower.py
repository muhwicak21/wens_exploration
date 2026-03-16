#!/usr/bin/env python3
"""
WENS Waypoint Follower
Follows a list of pre-defined waypoints loaded from a YAML config file.
"""

import rospy
import math
import yaml
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest


class WaypointFollower:
    """Follows a sequence of GPS / local-frame waypoints via MAVROS."""

    def __init__(self):
        rospy.init_node('wens_waypoint_follower', anonymous=True)

        self.waypoint_tolerance = rospy.get_param('~waypoint_tolerance', 0.5)
        self.hover_time = rospy.get_param('~hover_time', 2.0)
        self.default_altitude = rospy.get_param('~default_altitude', 3.0)
        waypoints_param = rospy.get_param('~waypoints', [])

        self.current_state = State()
        self.current_pose = PoseStamped()
        self.waypoints = self._load_waypoints(waypoints_param)
        self.current_wp_index = 0

        self.local_pos_pub = rospy.Publisher(
            'mavros/setpoint_position/local',
            PoseStamped,
            queue_size=10
        )

        rospy.Subscriber('mavros/state', State, self._state_cb)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self._pose_cb)

        rospy.wait_for_service('mavros/cmd/arming')
        rospy.wait_for_service('mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

        rospy.loginfo("WaypointFollower initialized with %d waypoints.", len(self.waypoints))

    def _state_cb(self, msg):
        self.current_state = msg

    def _pose_cb(self, msg):
        self.current_pose = msg

    def _load_waypoints(self, waypoints_param):
        poses = []
        for wp in waypoints_param:
            pose = PoseStamped()
            pose.pose.position.x = wp.get('x', 0.0)
            pose.pose.position.y = wp.get('y', 0.0)
            pose.pose.position.z = wp.get('z', self.default_altitude)
            pose.pose.orientation.w = wp.get('yaw_w', 1.0)
            poses.append(pose)
        return poses

    def _dist(self, wp):
        dx = self.current_pose.pose.position.x - wp.pose.position.x
        dy = self.current_pose.pose.position.y - wp.pose.position.y
        dz = self.current_pose.pose.position.z - wp.pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def run(self):
        rate = rospy.Rate(20)

        if not self.waypoints:
            rospy.logwarn("No waypoints loaded. Exiting.")
            return

        # Pre-populate setpoint stream
        first_wp = self.waypoints[0]
        for _ in range(100):
            first_wp.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(first_wp)
            rate.sleep()

        # Arm and switch to OFFBOARD
        mode_req = SetModeRequest()
        mode_req.custom_mode = 'OFFBOARD'
        self.set_mode_client.call(mode_req)

        arm_req = CommandBoolRequest()
        arm_req.value = True
        self.arming_client.call(arm_req)

        rospy.loginfo("Starting waypoint following...")

        while not rospy.is_shutdown():
            if self.current_wp_index >= len(self.waypoints):
                rospy.loginfo("All waypoints reached.")

                mode_req.custom_mode = 'AUTO.LAND'
                self.set_mode_client.call(mode_req)
                break

            wp = self.waypoints[self.current_wp_index]
            wp.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(wp)

            if self._dist(wp) < self.waypoint_tolerance:
                rospy.loginfo("Reached waypoint %d / %d.",
                              self.current_wp_index + 1, len(self.waypoints))
                rospy.sleep(self.hover_time)
                self.current_wp_index += 1

            rate.sleep()


if __name__ == '__main__':
    try:
        node = WaypointFollower()
        node.run()
    except rospy.ROSInterruptException:
        pass
