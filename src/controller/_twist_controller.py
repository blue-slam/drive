import numpy as np
import rospy

from geometry_msgs.msg import Twist
from _drive_controller import DriveController


class TwistController:

    def __init__(self, params):
        self._drive_controller = DriveController(params)

        self._d = rospy.get_param(params['lowlevel_name'] + '/geometry/d')
        self._r = rospy.get_param(params['lowlevel_name'] + '/geometry/r')

        self._wheel_vel = np.zeros([2, 1])

        self._twist_sub = rospy.Subscriber('/cmd_vel', Twist, self._twist_handler)

        self._drive_controller.drive(self._wheel_vel)

    def _twist_handler(self, msg):
        self._wheel_vel = (msg.linear.x + np.array([[-1],[1]]) * (msg.angular.z * self._d) / 2) / self._r
        # self._wheel_vel[0, 0] = (msg.linear.x - (msg.angular.z * (self._d / 2))) / self._r
        # self._wheel_vel[1, 0] = (msg.linear.x + (msg.angular.z * (self._d / 2))) / self._r

    def drive(self):
        self._drive_controller.drive(self._wheel_vel)

    def shutdown(self):
        self._twist_sub = None
