#!/usr/bin/env python
import Adafruit_ADS1x15

class SharpScanner:
    GAIN = 1
    
    def __init__(self):
        self.adc = Adafruit_ADS1x15.ADS1115()

    def read_adc(self, i):
        sum = 0
        for i in range(0, 8):
            sum += self.adc.read_adc(i, gain=GAIN)

        return sum/8

    def scan(self):
        # Scan straight
        min_dist = self.read_adc(0)

        # Scan right

        # Scan left
        
        print("Distance sensor", values)
        
        min_dist = 10000
        if split_values[0] < min_dist:
            min_dist = split_values[0]
        if split_values[1] < min_dist:
            min_dist = split_values[1]
        if split_values[2] < min_dist:
            min_dist = split_values[2]

        return min_dist

if __name__ == "__main__":
    scanner = SharpScanne()
    scanner.scan()
