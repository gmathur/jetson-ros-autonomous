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

VERTICAL_ANGLE_THRESHOLD = 0.03
CONTIGUOUS_VERTICAL_PERIODS_THRESHOLD = 7

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
    def __init__(self, driver):
        self.reset_imu = True
        self.reset_euler = True
        self.baseline_imu = None
        self.baseline_euler = None
        self.last_euler = None
        self.last_imu = None
        self.last_robot_state = RobotState.STOP
        self.start_vertical_angle = None
        self.analyzing_vertical_start = self.get_current_time()
        self.tracked_imu_periods = [0] * 20
        self.tracked_imu_period_index  = 0
        self.driver = driver

        rospy.Subscriber("imu/data", Imu, self.imu_callback, queue_size=1)
        rospy.Subscriber("imu/euler", Vector3, self.euler_callback, queue_size=1)
        rospy.Subscriber("robot_commands", RobotCmd, self.robot_cmd_callback, queue_size=1)

    def robot_cmd_callback(self, data):
        self.last_robot_state = data.cmd

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

    def get_current_time(self):
        return int(round(time.time() * 1000))

    def get_threshold_exeeded_periods(self):
        positive = 0
        negative = 0

        count = 0
        current_index = self.tracked_imu_period_index
        while(count < 15):
            if self.tracked_imu_periods[current_index] > 0:
                positive += 1
            elif self.tracked_imu_periods[current_index] < 0:
                negative += 1

            count += 1
            current_index -= 1
            if current_index < 0:
                current_index = 19

        return positive if positive > negative else negative

    def do_emergency_checks(self, data):
        if self.start_vertical_angle is None or self.last_robot_state == RobotState.STOP:
            self.start_vertical_tracking(data)
            return

        if self.get_current_time() - self.analyzing_vertical_start < 0.01:
            return

        if self.start_vertical_angle is None:
            return

        # Time is up
        delta = abs(math.sin(data.y)) - self.start_vertical_angle

        if abs(delta) > VERTICAL_ANGLE_THRESHOLD:
            self.tracked_imu_periods[self.tracked_imu_period_index] = 1 if delta > 0 else -1
            
            contiguous_vertical_imu_periods = self.get_threshold_exeeded_periods()
            rospy.loginfo("Current vertical delta %f self.contiguous_vertical_imu_periods %d", delta, contiguous_vertical_imu_periods)
            if contiguous_vertical_imu_periods >= CONTIGUOUS_VERTICAL_PERIODS_THRESHOLD:
                rospy.logerr("********************** EMERGENCY STOP! Vertical change %f **********************", delta)
                self.driver.do_emergency_stop()
        else:
            self.tracked_imu_periods[self.tracked_imu_period_index] = 0
        
        self.tracked_imu_period_index = (self.tracked_imu_period_index + 1) % 20
        self.start_vertical_tracking(data)

    def start_vertical_tracking(self, data):
        self.start_vertical_angle = abs(math.sin(data.y))
        self.analyzing_vertical_start =  self.get_current_time()

    def euler_callback(self, data):
        # Do emergency checks
        self.do_emergency_checks(data)

        if self.reset_euler:
            self.reset_euler = False
            self.baseline_euler = data

        self.last_euler = data

    def imu_callback(self, data):
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
            rospy.loginfo("Returning false for IMU data %s" % (max_imu))
            return False

    def get_angular_change(self):
        if self.baseline_euler is None or self.last_euler is None:
            return 0.0
        return angle_between((self.baseline_euler.x, self.baseline_euler.y, 0),
            (self.last_euler.x, self.last_euler.y, 0))

    def should_use_imu(self):
        if self.last_imu and self.last_imu.header.stamp.secs >= rospy.Time.now().secs - 1:
            return True
        else:
            return False

