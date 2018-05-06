#!/usr/bin/env python
import rospy
import time
import math
import RPi.GPIO as GPIO
from std_msgs.msg import Int16
from robot_state import RobotState
import pigpio
from std_msgs.msg import Int16, String

class EncoderCounter:
    def __init__(self, pi, pin, topic):
        self.pi = pi
        self.pin = pin
        self.count = 0
        self.topic = topic

        self.pi.set_mode(pin, pigpio.INPUT)
        self.pi.set_pull_up_down(pin, pigpio.PUD_DOWN)
        self.pi.callback(pin, pigpio.RISING_EDGE, self.detect)

        self.pub = rospy.Publisher(topic, Int16, queue_size=1)

        self.increment_counter = True

    def set_counter_positive(self, increment):
        self.increment_counter = increment

    def detect(self, gpio, level, tick):
        if self.increment_counter:
            if self.count == 32768:
                self.count = 0 # Wrap around
            else:
                self.count += 1
        else:
            if self.count == 0:
                self.count = 32768 # Wrap around
            else:
                self.count -= 1

        if self.count % 10 == 0:
            rospy.logdebug("%s encoder count %d", self.topic, self.count)

    def publish(self):
        counter = Int16(self.count)
        self.pub.publish(counter)

class RobotEncoderController:
    def __init__(self):
        self.pi = pigpio.pi()

        self.lencoder = EncoderCounter(self.pi, 23, 'lwheel')
        self.rencoder = EncoderCounter(self.pi, 24, 'rwheel')

        rospy.Subscriber("robot_commands", String, self.set_state, queue_size=1)

    def set_state(self, state):
        if state == RobotState.FORWARD:
            self.lencoder.set_counter_positive(True)
            self.rencoder.set_counter_positive(True)
        elif state == RobotState.REVERSE:
            self.lencoder.set_counter_positive(False)
            self.rencoder.set_counter_positive(False)
        elif state == RobotState.LEFT:
            self.lencoder.set_counter_positive(False)
            self.rencoder.set_counter_positive(True)
        elif state == RobotState.FORWARD:
            self.lencoder.set_counter_positive(True)
            self.rencoder.set_counter_positive(False)

        self.lencoder.publish()
        self.rencoder.publish()

    def stop(self):
        self.pi.stop()

    def spin(self):
        rospy.loginfo("Encoder spinning")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.lencoder.publish()
            self.rencoder.publish()
        
            rate.sleep()
    
if __name__ == "__main__":
    controller = RobotEncoderController()
    try:
        rospy.init_node('wheel_encoder')
        controller.spin()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
        pass

    controller.stop()
