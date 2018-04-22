#!/usr/bin/env python
import random
import threading
import time
from dimension_driver import DimensionDriver
from robot_state import RobotState, CommandSource
from distance_scanner import DistanceScanner

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
    TURN_TIME = 0.8

    def __init__(self):
        self.driver = DimensionDriver(128, '/dev/serial0')
        self.state_tracker = RobotStates()
        self.scanner = DistanceScanner()
        self.state_tracker.add(RobotState.STOP)
        self.forward_speed = 127
        self.reverse_speed = 100
        self.turn_speed = 100

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

    def scan(self):
        min_dist = self.scanner.scan()

        # If straight is ok - keep going
        if min_dist < 10:
            self.execute_cmd(RobotState.STOP)
            self.check_obstacles()
            threading.Timer(0.01, self.scan)
        else:
            self.forward()
            threading.Timer(0.1, self.scan)

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

    def turn_left(self, run_time = TURN_TIME):
        self.execute_cmd(RobotState.LEFT)
        time.sleep(run_time)
        self.execute_cmd(RobotState.STOP)

    def turn_right(self, run_time = TURN_TIME):
        self.execute_cmd(RobotState.RIGHT)
        time.sleep(run_time)
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

