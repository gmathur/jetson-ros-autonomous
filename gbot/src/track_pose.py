#!/usr/bin/env python

import time
import math
import rospy
import sys
import signal
import numpy as np
from threading import Thread
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int16
from robot_state import RobotState
from gbot.msg import RobotCmd

class TrackPose:
    def __init__(self, driver):
        self.driver = driver
        self.last_robot_cmd = None
        self.robot_cmd_start_pose = None
        self.stop_tracking()

        rospy.Subscriber("pose2D", Pose2D, self.pose_callback, queue_size=1)
        rospy.Subscriber("robot_commands", RobotCmd, self.robot_cmd_callback, queue_size=1)
        self.manual_mode_pub = rospy.Publisher("manual_override", Int16, queue_size=1)

    def clear(self):
        self.start_pose = None
        self.last_pose = None

    def start_tracking(self):
        self.tracking = True

    def get_current_angle(self):
        if self.last_pose is None or self.start_pose is None:
            return 0.0

        diff = abs(self.last_pose.theta - self.start_pose.theta) % (2 * 3.142)
        return min(diff, (2 * 3.142 - diff))

    def pose_callback(self, data):
        self.do_stuck_checks(data)
        self.track_angle(data)

    def track_angle(self, data):
        self.last_pose = data
        if not self.tracking:
            return

        if self.start_pose is None:
            self.start_pose = data

    def stop_tracking(self):
        self.tracking = False
        self.clear()

    def do_stuck_checks(self, data):
        if self.last_robot_cmd and self.last_robot_cmd.header.stamp.secs > (time.time() - 2.0) or \
                self.robot_cmd_start_pose is None or \
                self.last_pose is None:
            return

        delta_x = abs(self.last_pose.x - self.robot_cmd_start_pose.x)
        delta_y = abs(self.last_pose.y - self.robot_cmd_start_pose.y)
        if self.last_robot_cmd.cmd in RobotState.MOTION_STATES and \
            delta_x < 0.1 and delta_y < 0.1:
            rospy.logwarn("Think we are stuck. state %s delta x %f delta y %f", self.last_robot_cmd.cmd,
                delta_x, delta_y)
            self.unstuck()

    def unstuck(self):
        data = Int16()
        data.data = 1
        self.manual_mode_pub.publish(data)

        self.driver.reverse(delay=2.0)
        self.driver.random_turn()

        data.data = 0
        self.manual_mode_pub.publish(data)

    def robot_cmd_callback(self, data):
        if self.last_robot_cmd and self.robot_cmd_start_pose and \
                self.last_robot_cmd.cmd == data.cmd:
            # Only interested in change of commands - remember each command
            # is issued periodically
            return
            
        self.last_robot_cmd = data
        self.robot_cmd_start_pose = self.last_pose

if __name__== "__main__":
    rospy.init_node("pose_track_node")

    track_imu = TrackPose()
    rospy.spin()
