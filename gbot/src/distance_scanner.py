#!/usr/bin/env python
import rospy
import time
from ultrasonic_scanner import UltrasonicScanner
from gbot.msg import Proximity

class DistanceScanner:
    def __init__(self):
        self.scanner = UltrasonicScanner()
#        self.scanner = SharpScanner()
        self.pub = rospy.Publisher('proximity', Proximity, queue_size=1)

    def spin(self):
        while not rospy.is_shutdown():
            # Scan straight
            distances = self.scanner.scan()
    
            if distances[0] == -1:
                rospy.logerr("Error in ultrasonic readings - retrying")
                continue

            rospy.loginfo("Distances %s" % (distances))

            msg = Proximity()
            msg.header.stamp = rospy.Time.now()
            msg.left = distances[0]
            msg.right = distances[0]
            msg.straight = distances[0]
            
            self.pub.publish(msg)
            time.sleep(0.05)

if __name__ == "__main__":
    rospy.init_node('proximity_node')

    scanner = DistanceScanner()
    scanner.spin()
