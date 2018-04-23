#!/usr/bin/env python

import sys
import rospy
import signal
from threading import Thread
from sensor_msgs.msg import Imu

class TrackImu:
    def __init__(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("imu/data", Imu, self.callback)
        self.max_imu = Imu()
        self.reset_imu = True
        self.baseline_imu = None

    def reset(self):
        self.max_imu.linear_acceleration.x = 0
        self.max_imu.linear_acceleration.y = 0
        self.max_imu.linear_acceleration.z = 0

        self.max_imu.angular_velocity.x = 0
        self.max_imu.angular_velocity.y = 0
        self.max_imu.angular_velocity.z = 0

        self.reset_imu = True

    def subtract_components(self, a, b, c):
        changed = False

        objects  = ["linear_acceleration", "angular_velocity"]
        components = ["x", "y", "z"]

        for j in objects:
            obja = getattr(a, j)
            objb = getattr(b, j)
            objc = getattr(c, j)

            for i in components:
                diff = round(getattr(objb, i) - getattr(obja, i), 3)
                if diff > getattr(objc, i):
                    setattr(objc, i, diff)
                    changed = True

        return changed

    def callback(self, data):
#        print(rospy.get_caller_id() + "I heard %s", data)
        if self.reset_imu:
            self.reset_imu = False
            self.baseline_imu = data

        updated = self.subtract_components(data, self.baseline_imu, self.max_imu)

        # Updated max
#        if updated:
#            print("Updated max IMU data %s", self.max_imu)

    def is_angular_change_significant(self):
        if self.max_imu.angular_velocity.x >= 0.01 or self.max_imu.angular_velocity.y >= 0.01 or \
           self.max_imu.angular_velocity.z >= 0.01:
            return True
        else:
            return False

    def is_linear_change_significant(self):
        if self.max_imu.linear_acceleration.x >= 0.01 or self.max_imu.linear_acceleration.y >= 0.01 or \
            self.max_imu.linear_acceleration.z >= 0.01:
            return True
        else:
            print("Returning false for IMU data %s", self.max_imu)
            return False
