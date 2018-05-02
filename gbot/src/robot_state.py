#!/usr/bin/env python

class Enum(set):
    def __getattr__(self, name):
        if name in self:
            return name
        raise AttributeError

RobotState = Enum(["FORWARD", "REVERSE", "LEFT",
    "RIGHT", "STOP"])

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

