#!/usr/bin/env python

import rospy

class SpeedTracker:
    MIN_SPEED = 107 # Multiple of 10
    MAX_SPEED = 127 # Multiple of 10

    def __init__(self):
        self.reset_forward_speed()
        self.reverse_speed = 100
        self.turn_speed = 80

    def reset_forward_speedreset_forward_speed(self):
        self.forward_speed = SpeedTracker.MIN_SPEED

    def reduce_forward_speed(self):
        if self.forward_speed > SpeedTracker.MIN_SPEED:
            self.forward_speed -= 10
            if self.forward_speed < SpeedTracker.MIN_SPEED:
                self.forward_speed = SpeedTracker.MIN_SPEED

    def increase_forward_speed(self):
        if self.forward_speed < SpeedTracker.MAX_SPEED:
            self.forward_speed += 10
            if self.forward_speed > SpeedTracker.MAX_SPEED:
                self.forward_speed = SpeedTracker.MAX_SPEED

    def adjust_speed(self, min_dist, last_dist):
        if min_dist > 70 or min_dist >= (last_dist - 2):
            self.increase_forward_speed()
            rospy.loginfo("min dist: %d, last dist %d, Increased speed to %d" % (min_dist, last_dist, self.forward_speed))
        else:
            self.reduce_forward_speed()
            rospy.loginfo("min dist: %d, last dist %d. Decreased speed to %d" % (min_dist, last_dist, self.forward_speed))


