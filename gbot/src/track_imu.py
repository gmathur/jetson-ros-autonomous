#!/usr/bin/env python

import time
import math
import rospy
import sys
import signal
import numpy as np
from threading import Thread
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from robot_state import RobotState
from gbot.msg import RobotCmd
from tf.transformations import euler_from_quaternion

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

class TrackImu:
    def __init__(self):
        self.stop_tracking()

        rospy.Subscriber("imu/euler", Vector3, self.euler_callback, queue_size=1)

    def clear(self):
        self.angular_change = 0.0
        self.baseline_euler = None
        self.last_euler = None

    def start_tracking(self):
        self.tracking = True
        self.clear()

    def get_current_angle(self):
        self.angular_change += abs(self.get_angular_change(self.baseline_euler, self.last_euler))

        self.baseline_euler = self.last_euler
        return self.angular_change

    def euler_callback(self, data):
        if not self.tracking:
            return

        if self.baseline_euler is None:
            self.baseline_euler = data

        self.last_euler = data

    def get_angular_change(self, baseline_euler, next_euler):
        if baseline_euler is None or next_euler is None:
            return 0.0
        return angle_between((baseline_euler.x, baseline_euler.y, 0),
            (next_euler.x, next_euler.y, 0))

    def stop_tracking(self):
        self.tracking = False
        self.clear()

    def should_use_imu(self):
        return True

if __name__== "__main__":
    rospy.init_node("imu_node")

    track_imu = TrackImu()
    rospy.spin()
