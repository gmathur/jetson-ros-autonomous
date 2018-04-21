#!/usr/bin/env python

class RobotState:
    FORWARD = 1
    REVERSE = 2
    LEFT = 3
    RIGHT = 4
    STOP = 5

class CommandSource:
    DISTANCE_SENSOR = 1
    USER = 2
    DRIVER = 3
    ARDUINO = 4
