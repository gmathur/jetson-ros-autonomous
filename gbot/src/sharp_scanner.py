#!/usr/bin/env python
import Adafruit_ADS1x15

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

if __name__ == "__main__":
    scanner = SharpScanner()
    dist = scanner.scan()
    print dist
