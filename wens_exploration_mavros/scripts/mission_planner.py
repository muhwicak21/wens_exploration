#!/usr/bin/env python3
"""
mission_planner.py

Mission planner for WENS UAV exploration.
Provides a higher-level interface for planning and executing exploration missions,
including boustrophedon (lawnmower) coverage patterns and random walk exploration.
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import Bool, String


class MissionPlanner:
    def __init__(self):
        rospy.init_node("mission_planner", anonymous=False)

        # Parameters
        self.mission_type = rospy.get_param("~mission_type", "frontier")  # frontier, coverage, random
        self.area_x_min = rospy.get_param("~area_x_min", -10.0)
        self.area_x_max = rospy.get_param("~area_x_max", 10.0)
        self.area_y_min = rospy.get_param("~area_y_min", -10.0)
        self.area_y_max = rospy.get_param("~area_y_max", 10.0)
        self.flight_altitude = rospy.get_param("~flight_altitude", 2.5)
        self.coverage_spacing = rospy.get_param("~coverage_spacing", 3.0)
        self.auto_start = rospy.get_param("~auto_start", False)

        # State
        self.current_pose = PoseStamped()
        self.mavros_state = State()

        # Subscribers
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self._pose_cb)
        rospy.Subscriber("mavros/state", State, self._state_cb)
        rospy.Subscriber("exploration/mission_done", Bool, self._mission_done_cb)

        # Publishers
        self.mission_start_pub = rospy.Publisher(
            "exploration/mission_start", Bool, queue_size=10
        )
        self.setpoint_pub = rospy.Publisher(
            "mavros/setpoint_position/local", PoseStamped, queue_size=10
        )
        self.planner_status_pub = rospy.Publisher(
            "exploration/planner_status", String, queue_size=10
        )

        rospy.loginfo("[MissionPlanner] Initialized. Mission type: %s", self.mission_type)

        if self.auto_start:
            rospy.Timer(rospy.Duration(5.0), self._auto_start_cb, oneshot=True)

    def _pose_cb(self, msg):
        self.current_pose = msg

    def _state_cb(self, msg):
        self.mavros_state = msg

    def _mission_done_cb(self, msg):
        if msg.data:
            rospy.loginfo("[MissionPlanner] Mission completed.")
            self._publish_status("MISSION_DONE")

    def _auto_start_cb(self, event):
        rospy.loginfo("[MissionPlanner] Auto-starting mission.")
        self.start_mission()

    def _publish_status(self, status):
        msg = String()
        msg.data = status
        self.planner_status_pub.publish(msg)

    def generate_coverage_path(self):
        """Generate a boustrophedon (lawnmower) coverage path."""
        waypoints = []
        x = self.area_x_min
        direction = 1
        while x <= self.area_x_max:
            if direction > 0:
                y_start = self.area_y_min
                y_end = self.area_y_max
            else:
                y_start = self.area_y_max
                y_end = self.area_y_min

            waypoints.append((x, y_start, self.flight_altitude))
            waypoints.append((x, y_end, self.flight_altitude))
            x += self.coverage_spacing
            direction *= -1

        rospy.loginfo("[MissionPlanner] Generated coverage path with %d waypoints.",
                      len(waypoints))
        return waypoints

    def generate_random_walk(self, num_waypoints=20):
        """Generate a random walk exploration path within the defined area."""
        waypoints = []
        x = (self.area_x_min + self.area_x_max) / 2.0
        y = (self.area_y_min + self.area_y_max) / 2.0

        for _ in range(num_waypoints):
            angle = np.random.uniform(0, 2 * math.pi)
            step = np.random.uniform(1.0, 3.0)
            x = np.clip(x + step * math.cos(angle), self.area_x_min, self.area_x_max)
            y = np.clip(y + step * math.sin(angle), self.area_y_min, self.area_y_max)
            waypoints.append((x, y, self.flight_altitude))

        rospy.loginfo("[MissionPlanner] Generated random walk with %d waypoints.",
                      len(waypoints))
        return waypoints

    def start_mission(self):
        """Start the planned mission."""
        if not self.mavros_state.connected:
            rospy.logwarn("[MissionPlanner] FCU not connected. Cannot start mission.")
            return

        self._publish_status("MISSION_STARTING")

        if self.mission_type == "coverage":
            waypoints = self.generate_coverage_path()
            self._execute_waypoints(waypoints)
        elif self.mission_type == "random":
            waypoints = self.generate_random_walk()
            self._execute_waypoints(waypoints)
        elif self.mission_type == "frontier":
            # Frontier exploration is handled by exploration_controller.py
            rospy.loginfo("[MissionPlanner] Frontier exploration: delegating to exploration_controller.")
            msg = Bool()
            msg.data = True
            self.mission_start_pub.publish(msg)
        else:
            rospy.logerr("[MissionPlanner] Unknown mission type: %s", self.mission_type)

    def _execute_waypoints(self, waypoints):
        """Publish waypoints to the waypoint navigator."""
        # Signal mission start
        msg = Bool()
        msg.data = True
        self.mission_start_pub.publish(msg)
        self._publish_status("EXECUTING_WAYPOINTS")
        rospy.loginfo("[MissionPlanner] Executing %d waypoints.", len(waypoints))

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self._publish_status(
                "READY" if self.mavros_state.connected else "WAITING_FCU"
            )
            rate.sleep()


if __name__ == "__main__":
    try:
        planner = MissionPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
