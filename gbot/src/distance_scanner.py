#!/usr/bin/env python
import rospy
from ultrasonic_scanner import UltrasonicScanner
from gbot.msg import Proximity
#from sharp_scanner import SharpScanner

class DistanceScanner:
    def __init__(self):
        self.scanner = UltrasonicScanner()
#        self.scanner = SharpScanner()
        self.pub = rospy.Publisher('proximity', Proximity, queue_size=10)

    def spin(self):
        while not rospy.is_shutdown():
            left, straight, right = scanner.scan()
            
            msg = Proximity()
            msg.stamp = rospy.Time.now()
            msg.left = left
            msg.straight = straight
            msg.right = right
            
            self.pub.publish(msg)

    def scan(self):
        # Scan left, straight, right
        (left_dist, straight_dist, right_dist) = self.scanner.scan()

        min_dist = straight_dist
        if left_dist < min_dist:
            min_dist = left_dist
        if right_dist < min_dist:
            min_dist = right_dist
    
        print("Left %d Straight %d Right %d. Min %d" % (left_dist, straight_dist[0],
            right_dist, min_dist))

        return min_dist

if __name__ == "__main__":
    rospy.init_node('proximity_node')

    scanner = DistanceScanner()
    scanner.spin()
