#!/usr/bin/env python
import time
import math
import RPi.GPIO as GPIO
import pigpio

class EncoderCounter:
    def __init__(self, pi, pin):
        self.pi = pi
        self.pin = pin
        self.count = 0

        self.pi.set_mode(pin, pigpio.INPUT)
        self.pi.set_pull_up_down(pin, pigpio.PUD_UP)
        self.pi.callback(pin, pigpio.RISING_EDGE, self.detect)
        print("init")

    def detect(self, gpio, level, tick):
        self.count += 1

        if self.count % 10 == 0:
            print(self.count)

if __name__ == "__main__":
    pi = pigpio.pi()
    encoder = EncoderCounter(pi, 23)

    try:
	while True:
            pass
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
        pass

    pi.stop()
