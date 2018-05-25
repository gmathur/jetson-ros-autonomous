#!/usr/bin/env python
import rospy
import threading
import time
from driver import Driver
from robot_state import RobotState
from gbot.msg import Proximity, RobotCmd
from sensor_msgs.msg import LaserScan
from laser_scan_processor import LaserScanProcessor
from std_msgs.msg import Int16

DISTANCE_THRESHOLD = 0.30

class AutoPilot:
    def __init__(self, driver):
        self.driver = driver
        self.last_execution = time.time() - 100000
        self.laser_scan_processor = LaserScanProcessor()
        
        rospy.Subscriber("proximity", Proximity, self.proximity_callback, queue_size=1)
        rospy.Subscriber("scan", LaserScan, self.laser_scan_callback, queue_size=1)
        rospy.Subscriber("robot_commands", RobotCmd, self.robot_cmds_callback, queue_size=1)
        rospy.Subscriber("manual_override", Int16, self.manual_mode_callback, queue_size=5)

        self.last_robot_state = None
        self.last_laser_scan = None
        self.last_proximity_data = None
        self.last_laser_scan_processed = None
        self.last_proximity_data_processed = None
        self.manual_override = False
        self.processing = False

    def manual_mode_callback(self, data):
        self.manual_override = (data.data == 1)
        rospy.logwarn("Manual override %s", self.manual_override)

    def laser_scan_callback(self, data):
        self.last_laser_scan = data

    def proximity_callback(self, data):
        self.last_proximity_data = data

    def robot_cmds_callback(self, data):
        # We only want to track changes
        if self.last_robot_state and data.cmd == self.last_robot_state.cmd:
            return

        self.last_robot_state = data

    def spin(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if not self.manual_override:
                if not self.processing:
                    self.spin_drive()
            rate.sleep()

    def is_proximity_data_current(self):
        if self.last_proximity_data is None or \
            (self.last_proximity_data_processed and self.last_proximity_data.header.stamp <= self.last_proximity_data_processed.header.stamp):
            return False
        
        if self.last_robot_state and \
            self.last_robot_state.cmd in RobotState.HARD_TURN_STATES and \
            self.last_proximity_data.header.stamp < self.last_robot_state.header.stamp:
            return False

        return True

    def check_proximity_data_for_obstacles(self):
        if not self.is_proximity_data_current():
            return None, None

        self.last_proximity_data_processed = self.last_proximity_data
        min_dist = self.get_min_dist_from_proximity_data()
        if min_dist <= DISTANCE_THRESHOLD:
            return True, min_dist
        else:
            return False, min_dist

    def get_min_dist_from_proximity_data(self):
        min_dist = self.last_proximity_data.left if self.last_proximity_data.left < self.last_proximity_data.straight else self.last_proximity_data.straight
        min_dist = min_dist if min_dist < self.last_proximity_data.right else self.last_proximity_data.right
    
        return min_dist

    def is_laser_scan_current(self):
        if self.last_laser_scan is None or \
            (self.last_laser_scan_processed and self.last_laser_scan.header.stamp <= self.last_laser_scan_processed.header.stamp):
            return False

        if self.last_robot_state and \
            self.last_robot_state.cmd in RobotState.HARD_TURN_STATES and \
            self.last_laser_scan.header.stamp < self.last_robot_state.header.stamp:
            rospy.loginfo("last robot state %s self.last_laser_scan.header.stamp %s self.last_robot_state.header.stamp %s",
                    self.last_robot_state.cmd, self.last_laser_scan.header.stamp, self.last_robot_state.header.stamp)
            return False

        return True

    def check_laser_scan_for_obstacles(self):
        if not self.is_laser_scan_current():
            rospy.loginfo("Laser scan not current - returning")
            return None, None
        
        res, steering_angle = self.laser_scan_processor.steer(self.last_laser_scan)
        self.last_laser_scan_processed = self.last_laser_scan

        return res, steering_angle

    def spin_drive(self):
        try:
            self.processing = True

            # If straight is ok - keep going
            proximity_check, min_dist = self.check_proximity_data_for_obstacles()
            laser_scan_check, steering_angle = self.check_laser_scan_for_obstacles()
            if proximity_check is None and laser_scan_check is None:
                rospy.loginfo("No current data - returning")
                if self.driver.state_tracker.num_states() == 1:
                    self.driver.forward()
                return

            if proximity_check or laser_scan_check:
                self.obstacle_encountered()
            else:
                if steering_angle and (steering_angle < 3.132 or steering_angle > 3.152):
                    self.driver.turn_angle(steering_angle)
                    self.driver.speed_tracker.adjust_speed(min_dist, self.driver.state_tracker.get_last_dist())
                else:
                    self.driver.speed_tracker.adjust_speed(min_dist, self.driver.state_tracker.get_last_dist())
                    self.driver.forward()

            self.driver.state_tracker.set_last_dist(min_dist)
            self.last_execution = time.time()
        finally:
            self.processing = False

    def obstacle_encountered(self):
        rospy.loginfo("Obstacle encountered. Using laser scan to find where to go.")
        # Stop
        self.driver.stop()
        self.driver.state_tracker.set_last_dist(0)

        if self.last_laser_scan:
            self.pick_heading_from_scan()

    def pick_heading_from_scan(self):
        angle, left_obstacle, right_obstacle = self.laser_scan_processor.pick_heading(self.last_laser_scan)

        # Turn towards angle or if none is found, reverse out
        if angle is None:
            rospy.loginfo("Laser scan already being processed - dont do anything")
        elif angle == -1 or (left_obstacle == True and right_obstacle == True):
            rospy.loginfo("Based on laser scans, no angle found - angle %f reversing left_obstacle %s right_obstacle %s",
                    angle, left_obstacle, right_obstacle)
            
            self.driver.reverse()
            self.driver.random_turn()
        else:
            rospy.loginfo("Based on laser scans, turning towards %f left_obstacle %s, right_obstacle %s", 
                    angle, left_obstacle, right_obstacle)

            # Turn in that direction
            self.driver.turn_angle(angle, left_obstacle, right_obstacle)

if __name__ == '__main__':
    rospy.init_node('robot_pilot')

    driver = Driver()
    auto_pilot = AutoPilot(driver)
    try:
        driver.open()
        auto_pilot.spin()
    finally:
        driver.stop()

