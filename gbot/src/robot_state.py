#!/usr/bin/env python

class Enum(set):
    def __getattr__(self, name):
        if name in self:
            return name
        raise AttributeError

RobotState = Enum(["FORWARD", "REVERSE", "STEER_LEFT", "LEFT",
    "STEER_RIGHT", "RIGHT", "STOP"])
RobotState.HARD_TURN_STATES = [RobotState.LEFT, RobotState.RIGHT]
RobotState.LINEAR_STATES = [RobotState.FORWARD, RobotState.STOP,
        RobotState.STEER_LEFT, RobotState.STEER_RIGHT]
RobotState.MOTION_STATES = [RobotState.FORWARD, RobotState.STEER_LEFT,
        RobotState.STEER_RIGHT, RobotState.LEFT, RobotState.RIGHT]

class CommandSource:
    DISTANCE_SENSOR = 1
    USER = 2
    DRIVER = 3
    ARDUINO = 4

class RobotStateTracker:
    def __init__(self):
        self.states = [RobotState.STOP]
        self.dist = 0

    def add(self, state):
        if len(self.states) > 0 and \
                self.states[len(self.states)-1] == state:
            # Dont store multiples of same state in the list
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
        if dist is None:
            return

        self.dist = dist

    def get_last_dist(self):
        return self.dist

    def num_states(self):
        return len(self.states)
