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
