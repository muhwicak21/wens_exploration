#!/usr/bin/env python3
"""
WENS Offboard Control
Low-level offboard velocity / position control helper for MAVROS.
"""

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest


class OffboardControl:
    """
    Helper class for OFFBOARD mode control.
    Publishes a continuous setpoint stream so the FCU does not disarm.
    """

    def __init__(self):
        rospy.init_node('wens_offboard_control', anonymous=True)

        self.current_state = State()
        self.setpoint_pose = PoseStamped()
        self.setpoint_pose.pose.position.z = rospy.get_param('~hover_altitude', 2.0)
        self.setpoint_pose.pose.orientation.w = 1.0

        self.local_pos_pub = rospy.Publisher(
            'mavros/setpoint_position/local',
            PoseStamped,
            queue_size=10
        )
        self.vel_pub = rospy.Publisher(
            'mavros/setpoint_velocity/cmd_vel',
            TwistStamped,
            queue_size=10
        )

        rospy.Subscriber('mavros/state', State, self._state_cb)

        rospy.wait_for_service('mavros/cmd/arming')
        rospy.wait_for_service('mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

    def _state_cb(self, msg):
        self.current_state = msg

    def set_hover_position(self, x, y, z):
        self.setpoint_pose.pose.position.x = x
        self.setpoint_pose.pose.position.y = y
        self.setpoint_pose.pose.position.z = z

    def run(self):
        rate = rospy.Rate(20)
        last_request = rospy.Time.now()

        # Pre-populate setpoint stream
        for _ in range(100):
            self.setpoint_pose.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(self.setpoint_pose)
            rate.sleep()

        offb_mode_req = SetModeRequest()
        offb_mode_req.custom_mode = 'OFFBOARD'

        arm_req = CommandBoolRequest()
        arm_req.value = True

        while not rospy.is_shutdown():
            if self.current_state.mode != 'OFFBOARD' and \
                    (rospy.Time.now() - last_request) > rospy.Duration(5.0):
                if self.set_mode_client.call(offb_mode_req).mode_sent:
                    rospy.loginfo("OFFBOARD mode enabled.")
                last_request = rospy.Time.now()
            elif not self.current_state.armed and \
                    (rospy.Time.now() - last_request) > rospy.Duration(5.0):
                if self.arming_client.call(arm_req).success:
                    rospy.loginfo("Vehicle armed.")
                last_request = rospy.Time.now()

            self.setpoint_pose.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(self.setpoint_pose)
            rate.sleep()


if __name__ == '__main__':
    try:
        node = OffboardControl()
        node.run()
    except rospy.ROSInterruptException:
        pass
