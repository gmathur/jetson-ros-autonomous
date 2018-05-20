#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from dimension_driver import DimensionDriver

class DriverNode:
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
            self.driver.send_lmotor_command(data.data)
        else:
            self.driver.send_rmotor_command(data.data)

if __name__ == '__main__':
    rospy.init_node('driver_node')

    dimension_driver = DimensionDriver(128, '/dev/ttyUSB0')

    try:
        dimension_driver.open()
        driver_node = DriverNode(dimension_driver)

        rospy.spin()
    finally:
        dimension_driver.close()
