#!/usr/bin/env python
import rospy
import time
from dimension_driver import DimensionDriver
from driver import Driver
from manual_pilot import ManualPilot
from auto_pilot import AutoPilot

class PilotInit:
    def __init__(self):
        self.dimension_driver = DimensionDriver(128, '/dev/ttyUSB0')
        self.driver = Driver(self.dimension_driver)
        self.auto_pilot = AutoPilot(self.driver)
        
        self.manual_pilot = ManualPilot(self.dimension_driver)
        
    def open(self):
        self.driver.open()

    def start(self):
        rospy.spin()

    def close(self):
        self.driver.stop()
        self.dimension_driver.stop()

    
if __name__ == '__main__':
    rospy.init_node('robot_pilot')

    pilot = PilotInit()
    try:
        pilot.open()
        pilot.start()
    finally:
        pilot.close()

