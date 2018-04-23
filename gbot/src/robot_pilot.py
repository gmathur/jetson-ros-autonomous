#!/usr/bin/env python
import random
import threading
import time
from dimension_driver import DimensionDriver
from robot_state import RobotState, CommandSource
from distance_scanner import DistanceScanner
from track_imu import TrackImu

class RobotStates:
    def __init__(self):
        self.states = []

    def add(self, state):
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

class RobotPilot:
    TURN_TIME = 0.6

    def __init__(self):
        self.driver = DimensionDriver(128, '/dev/serial0')
        self.state_tracker = RobotStates()
        self.scanner = DistanceScanner()
        self.track_imu = TrackImu()
        self.state_tracker.add(RobotState.STOP)
        self.forward_speed = 127
        self.reverse_speed = 100
        self.turn_speed = 100
        self.last_dist = None

    def open(self):
        self.driver.open()

    def start(self):
        while True:
            self.scan()
            time.sleep(1)

    def close(self):
        self.driver.stop()
        self.state_tracker.add(RobotState.STOP)

        self.driver.close()

    def check_for_collision(self, min_dist):
        if self.last_dist:
            if abs(self.last_dist - min_dist) > 150:
                self.last_dist = min_dist
                return True

        self.last_dist = min_dist
        return False

    def scan(self):
        min_dist = self.scanner.scan()

        # If straight is ok - keep going
        if min_dist < 20 or self.check_for_collision(min_dist):
            self.execute_cmd(RobotState.STOP)
            self.check_obstacles()
        else:
            self.forward()
            if self.track_imu_for_time(0.2, angular=False):
                self.execute_cmd(RobotState.STOP)
                self.check_obstacles()
        
        threading.Timer(0.01, self.scan)

    def check_obstacles(self):
        # If last state was not turn right - turn right & scan
        if not self.state_tracker.check_state_exists(RobotState.RIGHT):
            self.turn_right()
        elif not self.state_tracker.check_state_exists(RobotState.LEFT):
            # Else if last state was not turn left - turn left + left & scan
            self.turn_left(run_time=2 * RobotPilot.TURN_TIME)
        elif not self.state_tracker.check_state_exists(RobotState.REVERSE):
            # Tried turning right and left. So we are wedged - back out
            self.turn_right()
            self.reverse()
        else:
            rand = random.random()

            if rand <= 0.5:
                self.turn_left()
            else:
                self.turn_right()

    def forward(self):
        self.execute_cmd(RobotState.FORWARD)

    def track_imu_for_time(self, run_time, angular=True):
        for i in range(0, int(run_time / 0.2)):
            self.track_imu.reset()
            time.sleep(0.2)
            if angular:
                if not self.track_imu.is_angular_change_significant():
                    # Turn isnt happening for whatever reason
                    print("Aborting turn as IMU isnt changing")
                    return True
            else:
                if not self.track_imu.is_linear_change_significant():
                    # Not moving forward
                    print("Stopping movement as IMU isnt changing")
                    return True

        return False

    def turn_left(self, run_time = TURN_TIME):
        self.execute_cmd(RobotState.LEFT)
        self.track_imu_for_time(run_time)
        self.execute_cmd(RobotState.STOP)

    def turn_right(self, run_time = TURN_TIME):
        self.execute_cmd(RobotState.RIGHT)
        self.track_imu_for_time(run_time)
        self.execute_cmd(RobotState.STOP)

    def reverse(self):
        self.execute_cmd(RobotState.REVERSE)
        time.sleep(0.5)
        self.execute_cmd(RobotState.STOP)

    def execute_cmd(self, cmd):
        print("Executing ", cmd)
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

