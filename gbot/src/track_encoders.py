#!/usr/bin/env python

import time
import math
import rospy
from std_msgs.msg import Int16, String
from robot_state import RobotState
from gbot.msg import RobotCmd

class TrackEncoders:
    def __init__(self, driver):
        self.driver = driver

        self.last_state = None
        self.last_time = time.time()
        self.lmotor_reading = 0
        self.rmotor_reading = 0
        
        rospy.Subscriber("robot_commands", RobotCmd, self.robot_state_callback, queue_size=1)
        rospy.Subscriber("lwheel", Int16, self.lwheel_callback, queue_size=1)
        rospy.Subscriber("rwheel", Int16, self.rwheel_callback, queue_size=1)

    def robot_state_callback(self, data):
        if self.last_state != data.cmd:
            self.last_state = data.cmd
            self.last_time = time.time()

    def lwheel_callback(self, data):
        self.process(data, self.lmotor_reading, True)

    def rwheel_callback(self, data):
        self.process(data, self.rmotor_reading, False)

    def process(self, data, motor_reading, is_left):
        if self.last_state is None:
            return

        if (time.time() - self.last_time) < 1:
            return

        if self.last_state == RobotState.FORWARD:
            if data.data == motor_reading and motor_reading > 0:
                rospy.logerr("Robot stuck - %s not moving", "lmotor" if is_left else "rmotor")
                self.driver.do_emergency_stop()

        if is_left:
            self.lmotor_reading = data.data
        else:
            self.rmotor_reading = data.data
        self.last_time = time.time()


