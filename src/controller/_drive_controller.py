import rospy
import numpy as np

from std_msgs.msg import Empty
from sensor_msgs.msg import BatteryState
from lowlevel.msg import MotorPosition
from lowlevel.msg import MotorCmd


class DriveController:

    def __init__(self, params):

        gear_ratio = rospy.get_param(params['lowlevel_name'] + '/motors/gear_ratio')
        encoder_ratio = rospy.get_param(params['lowlevel_name'] + '/motors/encoder_ratio')
        max_rpm = rospy.get_param(params['lowlevel_name'] + '/motors/max_rpm')
        max_volts = rospy.get_param(params['lowlevel_name'] + '/motors/max_volts')
        self._min_volts = rospy.get_param(params['lowlevel_name'] + '/motors/min_volts')
        r = rospy.get_param(params['lowlevel_name'] + '/geometry/r')

        self._max_cmd = rospy.get_param(params['lowlevel_name'] + '/motors/max_cmd')

        self._ke = np.full([2, 1], (2 * np.pi) / (encoder_ratio * gear_ratio))

        self._kp = params['drive_controller']['kp']
        self._ki = params['drive_controller']['ki']
        self._kd = params['drive_controller']['kd']

        self._ku = 0
        self._kv = (30 * (max_volts - self._min_volts)) / (max_rpm * np.pi)

        self._last_wheel_stamp = rospy.get_rostime()
        self._measured_wheel_vel = np.zeros([2, 1])
        self._measured_wheel_pos = np.zeros([2, 1])
        self._ready = 0

        self._last_drive_stamp = rospy.get_rostime()
        self._u = np.zeros([2, 1])
        self._error = np.zeros([2, 1])
        self._integral = np.zeros([2, 1])

        self._command_pub = rospy.Publisher(params['lowlevel_name'] + '/motors/cmd', MotorCmd, queue_size=10)
        self._reset_pub = rospy.Publisher(params['lowlevel_name'] + '/motors/reset', Empty, queue_size=10)

        # self._command_pub.publish(MotorCmd(0,0))
        self._reset_pub.publish(Empty())

        self._position_sub = rospy.Subscriber(params['lowlevel_name'] + '/motors/position', MotorPosition, self._position_handler)
        self._battery_sub = rospy.Subscriber(params['lowlevel_name'] + "/battery", BatteryState, self._battery_handler)

    def _battery_handler(self, msg):
        self._ku = self._max_cmd / msg.voltage

    def _position_handler(self, msg):
        new_position = np.array([[msg.left_count], [msg.right_count]]) * self._ke
        dt = (msg.header.stamp - self._last_wheel_stamp).to_sec()
        if dt > 0:
            self._measured_wheel_vel = (new_position - self._measured_wheel_pos) / (msg.header.stamp - self._last_wheel_stamp).to_sec()
            self._measured_wheel_pos = new_position
            self._last_wheel_stamp = msg.header.stamp
            self._ready += 1
        else:
            print('bad dt: {:f}={:f}-{:f}'.format(dt, msg.header.stamp.to_sec(), self._last_wheel_stamp.to_sec()))

    def drive(self, wheel_vel_des):

        if self._ready < 10:
            return

        drive_stamp = rospy.get_rostime()

        dt = (drive_stamp - self._last_drive_stamp).to_sec()
        error = wheel_vel_des - self._measured_wheel_vel
        integral = self._integral + error * dt
        derivative = (error - self._error) / dt

        v = self._kp * error + self._ki * self._integral + self._kd * derivative
        u = self._u + np.round(self._ku * ((v * self._kv) + (np.sign(v) * self._min_volts)))

        if np.abs(error[0, 0]) == 0 or np.abs(u[0, 0]) > self._max_cmd:
            integral[0, 0] = 0.0
            if v[0, 0] == 0:
                u[0, 0] = 0
        if np.abs(error[1, 0]) == 0 or np.abs(u[1, 0]) > self._max_cmd:
            integral[1, 0] = 0.0
            if v[1, 0] == 0:
                u[1, 0] = 0

        self._last_drive_stamp = drive_stamp
        self._u = u
        self._error = error
        self._integral = integral

        # print('vel_des: [{:f},{:f}] wheel_vel: [{:f},{:f}] dt: {:f} error: [{:f},{:f}] integral: [{:f},{:f}] derivative: [{:f},{:f}] v:[{:f},{:f}] u: [{:f},{:f}]'.
        #       format(wheel_vel_des[0, 0], wheel_vel_des[1, 0],
        #              self._measured_wheel_vel[0, 0], self._measured_wheel_vel[1, 0],
        #              dt,
        #              error[0, 0], error[1, 0],
        #              integral[0, 0], integral[1, 0],
        #              derivative[0, 0], derivative[1, 0],
        #              v[0, 0], v[1, 0],
        #              u[0, 0], u[1, 0]))

        motor_cmd = MotorCmd()
        motor_cmd.left_motor = int(u[0, 0])
        motor_cmd.right_motor = int(u[1, 0])
        self._command_pub.publish(motor_cmd)

    def shutdown(self):
        self._position_sub = None
        self._battery_sub = None
        self._command_pub = None
        self._reset_pub = None

    def get_wheel_vel(self):
        wheel_vel = self._measured_wheel_vel
        return wheel_vel

    def get_wheel_pos(self):
        wheel_pos = self._measured_wheel_pos
        return wheel_pos

