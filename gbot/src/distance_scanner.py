#!/usr/bin/env python
from ultrasonic_scanner import UltrasonicScanner

class DistanceScanner:
    def __init__(self):
        self.scanner = UltrasonicScanner()

    def scan(self):
        # Scan left, straight, right
        straight_dist = self.scanner.scan()
        left_dist = straight_dist
        right_dist = straight_dist

        min_dist = straight_dist
        if left_dist < min_dist:
            min_dist = left_dist
        if right_dist < min_dist:
            min_dist = right_dist
    
        print("Left %d Straight %d Right %d. Min %d" % (left_dist, straight_dist,
            right_dist, min_dist))

        return min_dist

if __name__ == "__main__":
    scanner = DistanceScanner()
    scanner.scan()
