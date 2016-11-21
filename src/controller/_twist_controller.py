from threading import Lock

import numpy as np
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from control_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

# from geometry_msgs.msg import Quaternion
# from geometry_msgs.msg import TransformStamped
# from tf.broadcaster import TransformBroadcaster

from drive.srv import DriveEngage

class TwistController:

    def __init__(self, robot_name):

        self._lock = Lock()
        self._b = rospy.get_param(robot_name + '/geometry/d')
        self._r = rospy.get_param(robot_name + '/geometry/r')
        self._drive_timeout = rospy.get_param('~drive_timeout')

        self._engaged = False

        self.wheel_pos_stamp = rospy.get_rostime()
        self._cmd_stamp = rospy.get_rostime()

        self.wheel_pos = np.zeros([2, 1])

        self._pose = np.zeros([3,1])
        self._vel = np.zeros([2, 1])

        self._wheel_vel_cmd = np.zeros([2, 1])

        self._joint_cmd = JointCommand()
        self._joint_cmd.name = ['left_motor_to_left_motor_output_shaft_joint', 'right_motor_to_right_motor_output_shaft_joint']
        self._joint_cmd.velocity = [0.0, 0.0]

        self._twist_sub = rospy.Subscriber('/cmd_vel', Twist, self._twist_handler)
        self._joint_state_sub = rospy.Subscriber('/joint_states', JointState, self._joint_state_handler)

        self._joint_cmd_pub = rospy.Publisher('/joint_cmds', JointCommand, queue_size=10)
        self._odometry_pub = rospy.Publisher('/odometry/encoder', Odometry, queue_size=10)
        self._buzzer_pub = rospy.Publisher('/commands/buzzer', String, queue_size=10)

        self._engage_service = rospy.Service('/commands/engage', DriveEngage, self._engage_drive)

        # self._odometry_broadcaster = TransformBroadcaster()

    def _joint_state_handler(self, msg):

        stamp = rospy.get_rostime()

        new_wheel_pos = np.array([[msg.position[0]], [msg.position[1]]]) * self._r
        delta_wheel_pos = new_wheel_pos - self.wheel_pos

        delta_theta = (delta_wheel_pos[1, 0] - delta_wheel_pos[0, 0]) / self._b
        delta_pos = np.sum(delta_wheel_pos) * 0.5
        dt = (msg.header.stamp - self.wheel_pos_stamp).to_sec()

        self.wheel_pos_stamp = msg.header.stamp
        self.wheel_pos = new_wheel_pos

        self._pose += np.array([[delta_pos * np.cos(self._pose[2, 0] + delta_theta * 0.5)],
                                [delta_pos * np.sin(self._pose[2, 0] + delta_theta * 0.5)],
                                [delta_theta]])
        self._vel = np.array([[delta_pos], [delta_theta]]) / dt

        # print('pose: [{:f},{:f},{:f}]'.format(self._pose[0, 0], self._pose[1, 0], self._pose[2, 0]))

        translation = np.append(self._pose[0:2], 0.0).tolist()
        rotation = quaternion_from_euler(0, 0, self._pose[2, 0])
        # self._odometry_broadcaster.sendTransform(translation, rotation, stamp, 'base_link', 'odom')

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = 'odom'
        odometry.child_frame_id = 'base_link'
        odometry.pose.pose.position.x = self._pose[0, 0]
        odometry.pose.pose.position.y = self._pose[1, 0]
        odometry.pose.pose.orientation.x = rotation[0]
        odometry.pose.pose.orientation.y = rotation[1]
        odometry.pose.pose.orientation.z = rotation[2]
        odometry.pose.pose.orientation.w = rotation[3]
        odometry.twist.twist.linear.x = self._vel[0, 0]
        odometry.twist.twist.angular.z = self._vel[1, 0]
        self._odometry_pub.publish(odometry)

    def _twist_handler(self, msg):
        with self._lock:
                self._cmd_stamp = rospy.get_rostime()
                self._wheel_vel_cmd = (msg.linear.x + np.array([[-1], [1]]) * (msg.angular.z * self._b) / 2) / self._r

    def _engage_drive(self, request):
        rospy.loginfo('Engaged: received request')
        with self._lock:
            self._engaged = not self._engaged

        if self._engaged:
            self._buzzer_pub.publish(String('v15>>g16R16g16'))
        else:
            self._buzzer_pub.publish(String('v15<<c16R16c16'))

        rospy.loginfo('Engaged: {}'.format(self._engaged))
        return self._engaged

    def update(self):
        with self._lock:
            if (rospy.get_rostime() - self._cmd_stamp).to_sec() < self._drive_timeout:
                self._joint_cmd.velocity[0] = self._wheel_vel_cmd[0, 0]
                self._joint_cmd.velocity[1] = self._wheel_vel_cmd[1, 0]
            else:
                self._joint_cmd.velocity[0] = 0.0
                self._joint_cmd.velocity[1] = 0.0

        if self._engaged:
            self._joint_cmd_pub.publish(self._joint_cmd)

    def shutdown(self):
        self._twist_sub = None
