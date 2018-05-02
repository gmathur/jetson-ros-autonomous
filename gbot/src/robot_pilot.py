#!/usr/bin/env python
import rospy
import random
import threading
import time
from dimension_driver import DimensionDriver
from robot_state import RobotState, CommandSource
from track_imu import TrackImu
from gbot.msg import Proximity
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan
from driver import Driver

class AutoPilot:
    def __init__(self, driver):
        self.driver = driver
        self.last_execution = time.time() - 100000
        
        rospy.Subscriber("proximity", Proximity, self.proximity_callback, queue_size=1)
        rospy.Subscriber("scan", LaserScan, self.scan_callback, queue_size=1)

    def check_for_collision(self, min_dist):
        if self.driver.state_tracker.dist:
            if self.driver.state_tracker.dist - min_dist > 300:
                rospy.loginfo("Massive reading change (current %d last %d) - possible collision", self.driver.state_tracker.dist, min_dist)
                return True

        return False

    def scan_callback(self, data):
        pass

    def proximity_callback(self, data):
        if data.stamp.secs < rospy.Time.now().secs - 2:
            rospy.logerr("Getting only old IMU messages. Stopping!")
            self.driver.stop()
            return

        min_dist = data.left
        if data.straight < min_dist:
            min_dist = data.straight
        if data.right < min_dist:
            min_dist = data.right

        # If straight is ok - keep going
        if min_dist <= 25 or self.check_for_collision(min_dist):
            self.obstacle_encountered()
        else:
            just_started = len(self.driver.state_tracker.states) == 2 or self.driver.state_tracker.get_last_state() == RobotState.STOP
            self.driver.track_imu.reset()
            self.driver.speed_tracker.adjust_speed(min_dist, self.driver.state_tracker.get_last_dist())
            self.driver.forward()
            time.sleep(0.2)

            if just_started and not self.driver.track_imu.is_linear_change_significant():
                rospy.loginfo("No change in IMU readings since starting movement")
                self.obstacle_encountered()
            
        self.driver.state_tracker.set_last_dist(min_dist)
        self.last_execution = time.time()

    def obstacle_encountered(self):
        self.driver.stop()
        self.driver.state_tracker.set_last_dist(0)
        self.check_obstacles()

    def check_obstacles(self):
        # If last state was not turn right - turn right & scan
        if not self.driver.state_tracker.check_state_exists(RobotState.RIGHT):
            self.driver.turn_right()
            return
        elif not self.driver.state_tracker.check_state_exists(RobotState.LEFT):
            # Else if last state was not turn left - turn left + left & scan
            self.driver.turn_left(turn_time=2 * Driver.TURN_TIME)
            return
        elif not self.driver.state_tracker.check_state_exists(RobotState.REVERSE):
            # Tried turning right and left. So we are wedged - back out
            self.driver.reverse()
        
        rand = random.random()
        if rand <= 0.5:
            self.driver.turn_left()
        else:
            self.driver.turn_right()

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


class PilotInit:
    def __init__(self):
        self.dimension_driver = DimensionDriver(128, '/dev/ttyUSB1')
        self.driver = Driver(self.dimension_driver)
        self.auto_pilot = AutoPilot(self.driver)
        
        self.manual_pilot = ManualPilot(self.dimension_driver)
        
    def open(self):
        self.driver.open()

    def start(self):
        self.driver.spin()

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

