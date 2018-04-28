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
            # Scan left, straight, right
            distances = self.scanner.scan()
    
            rospy.loginfo("Distances %s" % (distances))

            msg = Proximity()
            msg.stamp = rospy.Time.now()
            msg.left = distances[0]
            msg.right = distances[1]
            if distances[0] < distances[1]:
                msg.straight = distances[0]
            else:
                msg.straight = distances[1]
            
            self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node('proximity_node')

    scanner = DistanceScanner()
    scanner.spin()
