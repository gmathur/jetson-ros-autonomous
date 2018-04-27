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

class RobotStates:
    def __init__(self):
        self.states = []
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

class RobotPilot:
    TURN_TIME = 0.6
    MIN_SPEED = 60 # Multiple of 10
    MAX_SPEED = 120 # Multiple of 10

    def __init__(self):
        self.driver = DimensionDriver(128, '/dev/serial0')
        self.state_tracker = RobotStates()
        self.track_imu = TrackImu()
        self.state_tracker.add(RobotState.STOP)
        self.reset_forward_speed()
        self.reverse_speed = 100
        self.turn_speed = 80

        rospy.Subscriber("proximity", Proximity, self.proximity_callback, queue_size=1)
        rospy.Subscriber("imu/data", Imu, self.track_imu.imu_callback, queue_size=1)
        rospy.Subscriber("imu/euler", Vector3, self.track_imu.euler_callback, queue_size=1)
        self.last_execution = time.time() - 100000

    def open(self):
        self.driver.open()

    def start(self):
        rospy.spin()

    def close(self):
        self.driver.stop()
        self.state_tracker.add(RobotState.STOP)

        self.driver.close()

    def check_for_collision(self, min_dist):
        if self.state_tracker.dist:
            if self.state_tracker.dist - min_dist > 175:
                rospy.loginfo("Massive reading change (current %d last %d) - possible collision", self.state_tracker.dist, min_dist)
                return True

        return False

    def proximity_callback(self, data):
        if data.stamp.secs < rospy.Time.now().secs - 2:
            rospy.loginfo("Skipping old message")
            self.execute_cmd(RobotState.STOP)
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
            self.track_imu.reset()
            self.adjust_speed(min_dist)
            self.forward()
            time.sleep(0.2)

            if just_started and not self.track_imu.is_linear_change_significant():
                rospy.loginfo("No change in IMU readings since starting movement")
                self.obstacle_encountered()
            else:
                self.state_tracker.set_last_dist(min_dist)

        self.last_execution = time.time()

    def obstacle_encountered(self):
        self.execute_cmd(RobotState.STOP)
        self.state_tracker.set_last_dist(0)
        self.check_obstacles()

    def reset_forward_speed(self):
        self.forward_speed = RobotPilot.MIN_SPEED

    def reduce_forward_speed(self):
        if self.forward_speed > RobotPilot.MIN_SPEED:
            self.forward_speed -= 10

    def increase_forward_speed(self):
        if self.forward_speed < RobotPilot.MAX_SPEED:
            self.forward_speed += 10
            if self.forward_speed > 127:
                self.forward_speed = 127

    def adjust_speed(self, min_dist):
        if min_dist > 70 or min_dist >= (self.state_tracker.get_last_dist() - 2):
            self.increase_forward_speed()
            rospy.loginfo("min dist: %d, last dist %d, Increased speed to %d" % (min_dist, self.state_tracker.get_last_dist(), self.forward_speed))
        else:
            self.reduce_forward_speed()
            rospy.loginfo("min dist: %d, last dist %d. Decreased speed to %d" % (min_dist, self.state_tracker.get_last_dist(), self.forward_speed))

    def check_obstacles(self):
        # If last state was not turn right - turn right & scan
        if not self.state_tracker.check_state_exists(RobotState.RIGHT):
            self.turn_right()
            return
        elif not self.state_tracker.check_state_exists(RobotState.LEFT):
            # Else if last state was not turn left - turn left + left & scan
            self.turn_left(turn_time=2 * RobotPilot.TURN_TIME)
            return
        elif not self.state_tracker.check_state_exists(RobotState.REVERSE):
            # Tried turning right and left. So we are wedged - back out
            self.reverse()
        
        rand = random.random()
        if rand <= 0.5:
            self.turn_left()
        else:
            self.turn_right()

    def forward(self):
        self.execute_cmd(RobotState.FORWARD)

    def track_angular_imu_for_time(self, turn_time, turn_angle=1.57):
        start_time = time.time()

        self.track_imu.reset()
        #for i in range(0, int(turn_time / 0.1)):
        while(True):
            time.sleep(0.1)
            angular_change = abs(self.track_imu.get_angular_change())
            rospy.loginfo("Angular change %f" % (angular_change))
            if angular_change < 0.17:
                # Turn isnt happening for whatever reason
                rospy.loginfo("Aborting turn as IMU isnt changing")
                return True
            
            if angular_change > turn_angle:
                return False

            if (time.time() - start_time) > 2 * 1000:
                return True

    def turn_left(self, turn_time = TURN_TIME):
        self.execute_cmd(RobotState.LEFT)
        self.track_angular_imu_for_time(turn_time)
        self.execute_cmd(RobotState.STOP)

    def turn_right(self, turn_time = TURN_TIME):
        self.execute_cmd(RobotState.RIGHT)
        self.track_angular_imu_for_time(turn_time)
        self.execute_cmd(RobotState.STOP)

    def reverse(self):
        self.execute_cmd(RobotState.REVERSE)
        time.sleep(1)
        self.execute_cmd(RobotState.STOP)

    def execute_cmd(self, cmd):
        rospy.loginfo("Executing %s fwd speed: %d" % (cmd, self.forward_speed))
        self.state_tracker.add(cmd)
        
        if cmd == RobotState.FORWARD:
            self.driver.drive_forward(self.forward_speed)
        elif cmd == RobotState.REVERSE:
            self.driver.drive_backward(self.reverse_speed)
        elif cmd == RobotState.LEFT:
            self.driver.turn_left(self.turn_speed)
        elif cmd == RobotState.RIGHT:
            self.driver.turn_right(self.turn_speed)
        elif cmd == RobotState.STOP:
            self.driver.stop()
            self.reset_forward_speed()

if __name__ == '__main__':
    rospy.init_node('robot_pilot')

    pilot = RobotPilot()
    pilot.start()

