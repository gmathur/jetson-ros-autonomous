#!/usr/bin/env python
import Adafruit_ADS1x15
from servo import ServoControl

class SharpScanner:
    def __init__(self):
        self.adc = Adafruit_ADS1x15.ADS1115()

    def read_adc(self, i):
        sum = 0
        for k in range(0, 4):
            reading = self.adc.read_adc(i,gain=1) * (4.096 / 32768.0)
            sum += reading

        return sum / 4.0

    def scan(self):
        voltage = self.read_adc(0)
        dist = 27.86 * (voltage ** -1.15)

        return dist

class DistanceScanner:
    def __init__(self):
        self.scanner = SharpScanner()
        self.servo = ServoControl()
        self.servo.setAngle(90)

    def scan(self):
        # Scan straight
        straight_dist = self.scanner.scan()
        min_dist = straight_dist

        self.servo.setAngle(45)
        right_dist = self.scanner.scan()
        if right_dist < min_dist:
            min_dist = right_dist
 
        self.servo.setAngle(135)
        left_dist = self.scanner.scan()
        if left_dist < min_dist:
            min_dist = left_dist
        self.servo.setAngle(90)
        
        print("Left %d Straight %d Right %d. Min %d" % (left_dist, straight_dist, right_dist, min_dist))

        return min_dist

if __name__ == "__main__":
    scanner = SharpScanner()
    scanner.scan()
