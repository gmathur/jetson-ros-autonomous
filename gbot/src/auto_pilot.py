#!/usr/bin/env python
import rospy
import random
import threading
import time
from robot_state import RobotState
from gbot.msg import Proximity, RobotCmd
from sensor_msgs.msg import LaserScan
from laser_scan_processor import LaserScanProcessor

DISTANCE_THRESHOLD = 0.50

class AutoPilot:
    def __init__(self, driver):
        self.driver = driver
        self.last_execution = time.time() - 100000
        self.laser_scan_processor = LaserScanProcessor()
        
        rospy.Subscriber("proximity", Proximity, self.proximity_callback, queue_size=1)
        rospy.Subscriber("scan", LaserScan, self.laser_scan_callback, queue_size=1)
        rospy.Subscriber("robot_commands", RobotCmd, self.robot_cmds_callback, queue_size=1)

        self.last_robot_state = None
        self.last_laser_scan = None
        self.last_proximity_data = None
        self.last_laser_scan_processed = None
        self.last_proximity_data_processed = None

    def laser_scan_callback(self, data):
        self.last_laser_scan = data

    def proximity_callback(self, data):
        self.last_proximity_data = data

    def robot_cmds_callback(self, data):
        self.last_robot_state = data

    def spin(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.spin_drive()
            rate.sleep()

    def check_proximity_data_for_obstacles(self):
        if self.last_proximity_data is None or \
                (self.last_proximity_data_processed and self.last_proximity_data.header.stamp <= self.last_proximity_data_processed.header.stamp):
            return None, None
        
        if self.last_robot_state and \
            not (self.last_robot_state.cmd in [RobotState.FORWARD, RobotState.STOP] and \
                self.last_proximity_data.header.stamp >= self.last_robot_state.header.stamp):
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

    def check_laser_scan_for_obstacles(self):
        if self.last_laser_scan is None or \
                (self.last_laser_scan_processed and self.last_laser_scan.header.stamp <= self.last_laser_scan_processed.header.stamp):
            return None
        
        if self.last_robot_state and \
            not (self.last_robot_state.cmd in [RobotState.FORWARD, RobotState.STOP] and \
                self.last_laser_scan.header.stamp >= self.last_robot_state.header.stamp):
            return None

        res = self.laser_scan_processor.check_for_front_obstacles(self.last_laser_scan)
        self.last_laser_scan_processed = self.last_laser_scan

        return res

    def do_emergency_checks(self):
        if (self.last_proximity_data and self.last_proximity_data.header.stamp.secs < rospy.Time.now().secs - 1) and \
            (self.last_laser_scan and self.last_laser_scan.header.stamp.secs < rospy.Time.now().secs - 1):
            rospy.logerr("Getting only old proximity and laser scan messages. Stopping!")
            self.driver.do_emergency_stop()

    def spin_drive(self):
        self.do_emergency_checks()

        # If straight is ok - keep going
        proximity_check, min_dist = self.check_proximity_data_for_obstacles()
        laser_scan_check = self.check_laser_scan_for_obstacles()
        if proximity_check is None and laser_scan_check is None:
            rospy.loginfo("No data yet on new position - returning")
            return

        if proximity_check or laser_scan_check:
            self.obstacle_encountered()
        else:
            #just_started = len(self.driver.state_tracker.states) == 2 or \
            #        self.driver.state_tracker.get_last_state() == RobotState.STOP
            self.driver.track_imu.reset()
            self.driver.speed_tracker.adjust_speed(min_dist, self.driver.state_tracker.get_last_dist())
            self.driver.forward()
            time.sleep(0.2)

            #if just_started and self.driver.track_imu.should_use_imu() and \
            #        not self.driver.track_imu.is_linear_change_significant():
            #    rospy.loginfo("No change in IMU readings since starting movement")
            #    self.obstacle_encountered()
            
        self.driver.state_tracker.set_last_dist(min_dist)
        self.last_execution = time.time()

    def obstacle_encountered(self):
        # Stop
        self.driver.stop()
        self.driver.state_tracker.set_last_dist(0)

        rospy.loginfo("Using laser scan to find where to go.")
        self.process_laser_scan()

    def process_proximity(self):
        # If last state was not turn right - turn right & scan
        if not self.driver.state_tracker.check_state_exists(RobotState.RIGHT):
            self.driver.turn_right()
            return
        elif not self.driver.state_tracker.check_state_exists(RobotState.LEFT):
            # Else if last state was not turn left - turn left + left & scan
            self.driver.turn_angle(3.14)
            return
        elif not self.driver.state_tracker.check_state_exists(RobotState.REVERSE):
            # Tried turning right and left. So we are wedged - back out
            self.driver.reverse()
        
        self.random_turn()

    def random_turn(self):
        rand = random.random()
        if rand <= 0.5:
            self.driver.turn_left()
        else:
            self.driver.turn_right()

    def process_laser_scan(self):
        angle, left_obstacle, right_obstacle = self.laser_scan_processor.process_laser_scan(self.last_laser_scan)

        # Turn towards angle or if none is found, reverse out
        if angle is None:
            rospy.loginfo("Laser scan already being processed - dont do anything")
        elif angle == -1 or (left_obstacle and right_obstacle):
            rospy.loginfo("Based on laser scans, no angle found - reversing left_obstacle %s right_obstacle %s",
                    left_obstacle, right_obstacle)
            
            self.driver.reverse()
            self.random_turn()
        else:
            rospy.loginfo("Based on laser scans, turning towards %f left_obstacle %s, right_obstacle %s", 
                    angle, left_obstacle, right_obstacle)

            # Turn in that direction
            self.driver.turn_angle(angle, left_obstacle, right_obstacle)

