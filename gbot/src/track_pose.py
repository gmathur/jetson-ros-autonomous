#!/usr/bin/env python

import time
import math
import rospy
import sys
import signal
import numpy as np
from threading import Thread
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from robot_state import RobotState
from gbot.msg import RobotCmd

class TrackPose:
    def __init__(self):
        self.stop_tracking()

        rospy.Subscriber("pose2D", Pose2D, self.pose_callback, queue_size=1)

    def clear(self):
        self.start_theta = None
        self.last_theta = None

    def start_tracking(self):
        self.tracking = True

    def get_current_angle(self):
        if self.last_theta is None or self.start_theta is None:
            return 0.0
        return abs(self.last_theta - self.start_theta)

    def pose_callback(self, data):
        if not self.tracking:
            return

        if self.start_theta is None:
            self.start_theta = data.theta

        self.last_theta = data.theta

    def stop_tracking(self):
        self.tracking = False
        self.clear()

if __name__== "__main__":
    rospy.init_node("pose_track_node")

    track_imu = TrackPose()
    rospy.spin()
