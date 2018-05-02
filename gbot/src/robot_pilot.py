#!/usr/bin/env python
import rospy
import random
import threading
import time
from robot_state import RobotState, CommandSource
from track_imu import TrackImu
from gbot.msg import Proximity
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Vector3
from geometry_msgs.msg import LaserScan
from speed_tracker import SpeedTracker
from driver import Driver

class RobotStates:
    def __init__(self):
        self.states = [RobotState.STOP]
        self.dist = 0

    def add(self, state):
        if state == RobotState.STOP and len(self.states) > 0 and \
                self.states[len(self.states)-1] == state:
            # Dont store multiple STOP states in the list
            return

        self.states.append(state)
        if len(self.states) > 4:
            self.states.pop(0)

    def check_state_exists(self, state):
        return state in self.states

    def get_last_state(self):
        if len(self.states) == 0:
            return None
        else:
            return self.states[len(self.states)-1]

    def set_last_dist(self, dist):
        self.dist = dist

    def get_last_dist(self):
        return self.dist

class AutoPilot:
    def __init__(self, driver):
        self.driver = driver
        self.state_tracker = RobotStates()
        self.speed_tracker = SpeedTracker()
        self.last_execution = time.time() - 100000

    def check_for_collision(self, min_dist):
        if self.state_tracker.dist:
            if self.state_tracker.dist - min_dist > 300:
                rospy.loginfo("Massive reading change (current %d last %d) - possible collision", self.state_tracker.dist, min_dist)
                return True

        return False

    def scan_callback(self, data):
        pass

    def proximity_callback(self, data):
        if data.stamp.secs < rospy.Time.now().secs - 2:
            rospy.loginfo("Skipping old message")
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
            just_started = len(self.state_tracker.states) == 2 or self.state_tracker.get_last_state() == RobotState.STOP
            self.driver.track_imu.reset()
            self.speed_tracker.adjust_speed(min_dist, self.state_tracker.get_last_dist())
            self.driver.forward()
            time.sleep(0.2)

            if just_started and not self.pilot.track_imu.is_linear_change_significant():
                rospy.loginfo("No change in IMU readings since starting movement")
                self.obstacle_encountered()
            
        self.state_tracker.set_last_dist(min_dist)
        self.last_execution = time.time()

    def obstacle_encountered(self):
        self.driver.stop()
        self.state_tracker.set_last_dist(0)
        self.check_obstacles()

    def check_obstacles(self):
        # If last state was not turn right - turn right & scan
        if not self.state_tracker.check_state_exists(RobotState.RIGHT):
            self.driver.turn_right()
            return
        elif not self.state_tracker.check_state_exists(RobotState.LEFT):
            # Else if last state was not turn left - turn left + left & scan
            self.driver.turn_left(turn_time=2 * RobotPilot.TURN_TIME)
            return
        elif not self.state_tracker.check_state_exists(RobotState.REVERSE):
            # Tried turning right and left. So we are wedged - back out
            self.driver.reverse()
        
        rand = random.random()
        if rand <= 0.5:
            self.driver.turn_left()
        else:
            self.driver.turn_right()

class RobotPilot:
    TURN_TIME = 0.6
   
    def __init__(self):
        self.driver = Driver()
        self.obstacle_avoidance = AutoPilot(self.driver)
        
        rospy.Subscriber("proximity", Proximity, self.proximity_callback, queue_size=1)
        rospy.Subscriber("imu/data", Imu, self.track_imu.imu_callback, queue_size=1)
        rospy.Subscriber("imu/euler", Vector3, self.track_imu.euler_callback, queue_size=1)
        rospy.Subscriber("scan", LaserScan, self.obstacle_avoidance.scan_callback, queue_size=1)

    def open(self):
        self.driver.open()

    def start(self):
        self.driver.spin()

    def close(self):
        self.driver.stop()
        self.driver.close()

    
if __name__ == '__main__':
    rospy.init_node('robot_pilot')

    pilot = RobotPilot()
    try:
        pilot.open()
        pilot.start()
    finally:
        pilot.close()

