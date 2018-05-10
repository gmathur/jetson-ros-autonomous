#!/usr/bin/env python

import sys
import time
import pigpio

pi = pigpio.pi()

if not pi.connected:
   exit()

class ServoControl:
    def __init__(self, pin, pi, min_pos, max_pos):
        self.pin = pin
        self.pi = pi
        self.min_pos = min_pos
        self.max_pos = max_pos
        self.set_position(1500)

    def set_position(self, new_position):
        self.pi.set_servo_pulsewidth(self.pin, new_position)
        self.position = new_position
        time.sleep(0.01)

    def move_to_position(self, new_position):
        if new_position > self.max_pos:
            new_position = self.max_pos
        if new_position < self.min_pos:
            new_position = self.min_pos

        step = 5 if new_position > self.position else -5

        for pos in range(self.position, new_position, step):
            self.set_position(pos)

        self.stop()

    def get_current_position(self):
        return self.position

    def stop(self):
        self.pi.set_servo_pulsewidth(self.pin, 0)

    def close(self):
        self.move_to_position(1500)

class TiltControl(ServoControl):
    def __init__(self):
        super(TiltControl, self).__init__(12, pi, 800, 2000)

class PanControl(ServoControl):
    def __init__(self):
        super(PanControl, self).__init__(13, pi, 1150, 1850)

if __name__ == '__main__':

    tilt = TiltControl()
    try:
        tilt.move_to_position(800)
        time.sleep(1)
        tilt.move_to_position(2000)
        time.sleep(1)

        pan = PanControl()
        pan.move_to_position(1000)
        time.sleep(1)
        pan.move_to_position(2000)
        time.sleep(1)
    finally:
        tilt.close()
        pan.close()
        pi.stop()


