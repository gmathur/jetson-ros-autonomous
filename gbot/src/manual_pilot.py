#!/usr/bin/env python
import rospy

class ManualPilot:
    def __init__(self, driver):
        self.driver = driver
        
        rospy.Subscriber("lmotor", Int16, self.lmotor_callback, queue_size=1)
        rospy.Subscriber("rmotor", Int16, self.rmotor_callback, queue_size=1)

    def lmotor_callback(self, data):
        self.process(1, data)

    def rmotor_callback(self, data):
        self.process(2, data)

    def process(self, motor_num, data):
        if motor_num == 1:
            if data >= 0:
                self.driver.send_command(0, data)
            else:
                self.driver.send_command(4, data)
        else:
            if data >= 0:
                self.driver.send_command(0, data)
            else:
                self.driver.send_command(5, data)

