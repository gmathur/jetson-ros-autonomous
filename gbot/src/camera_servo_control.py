#!/usr/bin/env python

import rospy
from servo import TiltControl, PanControl
from std_msgs.msg import String

class CameraServoControl:
    def __init__(self):
        self.tilt = TiltControl()
        self.pan = PanControl()
        rospy.Subscriber("/camera_commands", String, self.cmd_callback, queue_size=10)

    def cmd_callback(self, data):
        rospy.logdebug("Got cmd %s tilt %d pan %d", data, self.tilt.get_current_position(), self.pan.get_current_position())
        cmd = data.data

        if cmd == "U":
            self.tilt.move_to_position(self.tilt.get_current_position() - 40)
        elif cmd == "D":
            self.tilt.move_to_position(self.tilt.get_current_position() + 40)
        elif cmd == "L":
            self.pan.move_to_position(self.pan.get_current_position() + 40)
        elif cmd == "R":
            self.pan.move_to_position(self.pan.get_current_position() - 40)

    def spin(self):
        rospy.spin()

    def close(self):
        self.tilt.close()
        self.pan.close()

if __name__ == '__main__':
    rospy.init_node('camera_pan_tilt')

    control = CameraServoControl()
    rospy.loginfo("init done")
    try:
        control.spin()
    finally:
        control.close()

