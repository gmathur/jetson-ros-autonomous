#!/usr/bin/env python

import time
import math
import rospy
import sys
import signal
import numpy as np
from threading import Thread
from robot_state import RobotState
from gbot.msg import RobotCmd
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16

VERTICAL_ANGLE_THRESHOLD = 0.04
FWD_CONTIGUOUS_VERTICAL_PERIODS_THRESHOLD = 5
TURN_CONTIGUOUS_VERTICAL_PERIODS_THRESHOLD = 24

class TrackVerticalPose:
    def __init__(self, driver):
        self.driver = driver
        self.start_vertical_angle = None
        self.last_robot_state = RobotState.STOP
        self.analyzing_vertical_start = self.get_current_time()
        self.tracked_imu_period_index = 0
        self.manual_override = False

        rospy.Subscriber("robot_commands", RobotCmd, self.robot_cmd_callback, queue_size=1)
        rospy.Subscriber("imu/euler", Vector3, self.euler_callback, queue_size=1)
        rospy.Subscriber("manual_override", Int16, self.manual_override_callback, queue_size=5)

    def manual_override_callback(self, data):
        self.manual_override = data.data == 1

    def euler_callback(self, data):
        # Do emergency checks
        self.do_emergency_checks(data)

    def robot_cmd_callback(self, data):
        if self.last_robot_state != data.cmd:
            self.reset_imu_tracking()
        self.last_robot_state = data.cmd

    def reset_imu_tracking(self):
        self.tracked_imu_periods = [0] * 20

    def get_current_time(self):
        return int(round(time.time() * 1000))

    def do_emergency_checks(self, data):
        if self.manual_override:
            return

        if self.start_vertical_angle is None or \
            self.last_robot_state == RobotState.STOP:
            self.start_vertical_tracking(data)
            return

        if self.get_current_time() - self.analyzing_vertical_start < 0.005:
            return

        if self.start_vertical_angle is None:
            return

        # Time is up
        delta = abs(math.sin(data.y)) - self.start_vertical_angle
        num_periods_threshold = TURN_CONTIGUOUS_VERTICAL_PERIODS_THRESHOLD \
                if self.last_robot_state in RobotState.HARD_TURN_STATES \
                    else FWD_CONTIGUOUS_VERTICAL_PERIODS_THRESHOLD

        if abs(delta) > VERTICAL_ANGLE_THRESHOLD:
            self.tracked_imu_periods[self.tracked_imu_period_index] = 1 if delta > 0 else -1
            
            contiguous_vertical_imu_periods = self.get_threshold_exeeded_periods()
            rospy.loginfo("Current vertical delta %f self.contiguous_vertical_imu_periods %d",
                    delta, contiguous_vertical_imu_periods)
            if contiguous_vertical_imu_periods >= num_periods_threshold:
                rospy.logerr("********************** EMERGENCY STOP! Vertical change %f **********************", delta)
                self.driver.do_emergency_stop()
        else:
            self.tracked_imu_periods[self.tracked_imu_period_index] = 0
        
        self.tracked_imu_period_index = (self.tracked_imu_period_index + 1) % 20
        self.start_vertical_tracking(data)

    def start_vertical_tracking(self, data):
        self.start_vertical_angle = abs(math.sin(data.y))
        self.analyzing_vertical_start =  self.get_current_time()

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
