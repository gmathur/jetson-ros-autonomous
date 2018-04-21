#!/usr/bin/env python
import serial
from dimension_driver import DimensionDriver
from robot_state import RobotState

class ArduinoDriver:
    def __init__(self, port):
        self.ser = serial.Serial(port=port,
                        baudrate = 115200)

    def open(self, driver):
        self.driver = driver
        if not self.ser.is_open:
            self.ser.open()

    def close(self):
        self.ser.close()

    def scan(driver):
        self.ser.write("S")
        got_cmd = ser.readline()
            
        return got_cmd
