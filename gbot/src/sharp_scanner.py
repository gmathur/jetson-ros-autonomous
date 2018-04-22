#!/usr/bin/env python
import Adafruit_ADS1x15

class SharpScanner:
    GAIN = 1
    
    def __init__(self):
        self.adc = Adafruit_ADS1x15.ADS1115()

    def read_adc(self, i):
        sum = 0
        for k in range(0, 8):
            sum += self.adc.read_adc(i, gain=SharpScanner.GAIN)

        return sum/8

    def scan(self):
        # Scan straight
        min_dist = self.read_adc(0)
        print("Straight dist", min_dist)

        # Scan right

        # Scan left
        
        
        return min_dist

if __name__ == "__main__":
    scanner = SharpScanner()
    scanner.scan()
