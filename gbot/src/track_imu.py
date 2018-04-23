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

    def callback(self, data):
        print(rospy.get_caller_id() + "I heard %s", data.data)

