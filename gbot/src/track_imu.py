#!/usr/bin/env python

import sys
import rospy
import signal
import numpy as np
from threading import Thread
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

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
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("imu/data", Imu, self.callback)
        rospy.Subscriber("imu/euler", Vector3, self.euler_callback)
        self.reset_imu = True
        self.reset_euler = True
        self.baseline_imu = None
        self.baseline_euler = None
        self.last_euler = None
        self.last_imu = None

    def reset(self):
        self.reset_imu = True
        self.reset_euler = True

    def subtract_vectors(self, a, b, c):
        changed = False
        components = ["x", "y", "z"]

        for i in components:
            diff = round(getattr(b, i) - getattr(a, i), 3)
            if diff > getattr(c, i):
                setattr(c, i, diff)
                changed = True

        return changed

    def subtract_components(self, a, b, c):
        changed = False

        objects  = ["linear_acceleration", "angular_velocity"]

        for j in objects:
            obja = getattr(a, j)
            objb = getattr(b, j)
            objc = getattr(c, j)

            changed |= self.subtract_vectors(obja, objb, objc)

        return changed

    def euler_callback(self, data):
        if self.reset_euler:
            self.reset_euler = False
            self.baseline_euler = data

        self.last_euler = data

    def callback(self, data):
#        print(rospy.get_caller_id() + "I heard %s", data)
        if self.reset_imu:
            self.reset_imu = False
            self.baseline_imu = data

        self.last_imu = data

    def is_linear_change_significant(self):
        if self.baseline_imu is None or self.last_imu is None:
            return 0.0

        max_imu = Imu()
        self.subtract_components(self.baseline_imu, self.last_imu, max_imu)
        if max_imu.linear_acceleration.x >= 0.01 or max_imu.linear_acceleration.y >= 0.01 or \
           max_imu.linear_acceleration.z >= 0.01:
            return True
        else:
            print("Returning false for IMU data %s", max_imu)
            return False

    def get_angular_change(self):
        if self.baseline_euler is None or self.last_euler is None:
            return 0.0
        return angle_between((self.baseline_euler.x, self.baseline_euler.y, 0),
            (self.last_euler.x, self.last_euler.y, 0))
