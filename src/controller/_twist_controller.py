from threading import Lock

import numpy as np
import rospy

from geometry_msgs.msg import Twist
from control_msgs.msg import JointCommand


class TwistController:

    def __init__(self, params):

        self._lock = Lock()
        self._d = rospy.get_param(params['robot_name'] + '/geometry/d')
        self._r = rospy.get_param(params['robot_name'] + '/geometry/r')

        self._wheel_vel = np.zeros([2, 1])
        self._joint_cmd = JointCommand()
        self._joint_cmd.name = ['left_motor_joint', 'right_motor_joint']
        self._joint_cmd.velocity = [0.0, 0.0]

        self._twist_sub = rospy.Subscriber('/cmd_vel', Twist, self._twist_handler)

        self._joint_cmd_pub = rospy.Publisher('/joint_cmds', JointCommand, queue_size=10)

    def _twist_handler(self, msg):
        with self._lock:
            self._wheel_vel = (msg.linear.x + np.array([[-1], [1]]) * (msg.angular.z * self._d) / 2) / self._r

    def update(self):
        with self._lock:
            self._joint_cmd.velocity[0] = self._wheel_vel[0, 0]
            self._joint_cmd.velocity[1] = self._wheel_vel[1, 0]

        self._joint_cmd_pub.publish(self._joint_cmd)

    def shutdown(self):
        self._twist_sub = None
