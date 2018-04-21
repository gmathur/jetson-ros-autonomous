#!/usr/bin/env python
import random
import threading
import time
from dimension_driver import DimensionDriver
from robot_state import RobotState, CommandSource
from arduino_driver import ArduinoDriver

class RobotStates:
    def __init__(self):
        self.states = []

    def add(self, state):
        self.states.append(state)
        if len(self.states) > 4:
            self.states.pop(0)

    def check_state_exists(self, state):
        if self.states.index(state) >= 0:
            return True
        else:
            return False

    def get_last_state(self):
        if len(self.states) == 0:
            return None
        else:
            return self.states[len(self.states)-1]

class RobotPilot:
    TURN_TIME = 0.2

    def __init__(self):
        self.arduino = ArduinoDriver('/dev/ttyUSB1')
        self.driver = DimensionDriver(128, '/dev/serial0')
        self.state_tracker = RobotStates()
        self.scanner = DistanceScanner(self.arduino)
        self.state_tracker.add(RobotState.STOP)
        self.forward_speed = 127
        self.reverse_speed = 100
        self.turn_speed = 100

    def open(self):
        self.driver.open()
        self.arduino.open(self)

        threading.Timer(1.0, self.scan)

    def close(self):
        self.timer.cancel()
        self.driver.stop()
        self.state_tracker.add(RobotState.STOP)

        self.driver.close()
        self.arduino.close()

    def scan(self):
        min_dist = self.scanner.scan()

        # If straight is ok - keep going
        if True:
            # TODO
            threading.Timer(1.0, self.scan)
        else:
            self.execute_cmd(RobotState.STOP)
            self.check_obstacles(left, right)
            threading.Timer(0.1, self.scan)

    def check_obstacles(self, left, right):
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

    def turn_left(self, run_time = RobotPilot.TURN_TIME):
        self.execute_cmd(RobotState.LEFT)
        time.sleep(run_time)
        self.execute_cmd(RobotState.STOP)

    def turn_right(self, run_time = RobotPilot.TURN_TIME):
        self.execute_cmd(RobotState.RIGHT)
        time.sleep(run_time)
        self.execute_cmd(RobotState.STOP)

    def reverse(self):
        self.execute_cmd(RobotState.REVERSE)
        time.sleep(0.5)
        self.execute_cmd(RobotState.STOP)

    def execute_cmd(self, cmd):
        print("Executing " + cmd)
        self.state_tracker.add(cmd)
        
        if cmd == RobotState.FORWARD:
            driver.drive_forward(self.forward_speed)
        elif cmd == RobotState.REVERSE:
            driver.drive_backward(self.reverse_speed)
        elif cmd == RobotState.LEFT:
            driver.turn_left(self.turn_speed)
        elif cmd == RobotState.RIGHT:
            driver.turn_right(self.turn_speed)
        elif cmd == RobotState.STOP:
            driver.stop()

class DistanceScanner:
    def __init__(self, arduino_driver):
        self.arduino_driver = arduino_driver

    def scan(self):
        values = self.arduino_driver.scan()

        split_values = values.split(",")
        
        min_dist = 10000
        if split_values[0] < min_dist:
            min_dist = split_values[0]
        if split_values[1] < min_dist:
            min_dist = split_values[1]
        if split_values[2] < min_dist:
            min_dist = split_values[2]

        return min_dist
