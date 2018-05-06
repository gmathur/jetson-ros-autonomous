#!/usr/bin/env python
import rospy
import random
import threading
import time
from robot_state import RobotState
from gbot.msg import Proximity
from sensor_msgs.msg import LaserScan
import numpy

MARGIN = 0.1
MIN_DIST = 0.3
FRONT_MIN_DIST = 0.1
SIDE_DIST_DELTA = 0.12

class ContiguousScanPoints:

    def __init__(self):
        self.pts = []
        self.start_angle = -1
        self.end_angle = -1
        self.median = None
        self.start_pt_num = -1

    def add(self, pt, angle, num):
        if len(self.pts) == 0:
            self.pts.append(pt)
            self.start_angle = angle
            self.start_pt_num = num
            return True
        
        last_pt = self.pts[len(self.pts)-1]
        if pt <= last_pt + (last_pt * MARGIN) and \
           pt >= last_pt - (last_pt * MARGIN):
            self.pts.append(pt)
            return True
        else:
            self.end_angle = angle
            return False

    def num(self):
        return len(self.pts)

    def get_median(self):
        if self.median is None:
            self.median = numpy.median(self.pts)

        return self.median

    def get_angle(self):
        return (self.start_angle + self.end_angle) / 2.0

class LaserScanProcessor:
    def __init__(self):
        self.pub = rospy.Publisher('scan_collisions', LaserScan, queue_size=1)

    def check_for_front_obstacles(self, laser_scan):
        angle_per_pt = (2 * 3.142) / len(laser_scan.ranges)
        
        # Calculate range of laser scan pts to evaluate
        right_max_range = int(0.7855 / angle_per_pt)
        left_min_range = len(laser_scan.ranges) - right_max_range

        num_matches = 0
        for i in range(0, right_max_range):
            if laser_scan.ranges[i] > 0 and \
                    laser_scan.ranges[i] < FRONT_MIN_DIST + i * (SIDE_DIST_DELTA / right_max_range):
                num_matches += 1

        for i in range(len(laser_scan.ranges)-1, left_min_range, -1):
            dist_delta = i - left_min_range
            if laser_scan.ranges[i] > 0 and \
                    laser_scan.ranges[i] < FRONT_MIN_DIST + dist_delta * (SIDE_DIST_DELTA / right_max_range):
                num_matches += 1

        if num_matches > 10:
            return True
        else:
            return False

    def process_laser_scan(self, laser_scan):
        contiguous_pts = [ContiguousScanPoints()]

        # Find largest distance in scan
        angle_per_pt = (2 * 3.142) / len(laser_scan.ranges)
        current_angle = 0
        
        # Find all contiguous points
        for i in range(len(laser_scan.ranges)):
            pt = laser_scan.ranges[i]
            if pt < laser_scan.range_min or pt > laser_scan.range_max:
                rospy.logdebug("Skipping laser pt at %f", current_angle)
                continue

            if not contiguous_pts[len(contiguous_pts)-1].add(pt, current_angle, i):
                new_contiguous_pts = ContiguousScanPoints()
                new_contiguous_pts.add(pt, current_angle, i)
                contiguous_pts.append(new_contiguous_pts)
            
            current_angle += angle_per_pt

        # Clean up the contiguous pts
        contiguous_pts = [ptset for ptset in contiguous_pts \
                                if ptset.num() > 5 and ptset.get_median() >= MIN_DIST]

        # Now look for largest pts
        contiguous_pts = sorted(contiguous_pts, key=lambda item: item.get_median(), reverse=True)
        
        # Pick the largest one that is not right behind
        for ptset in contiguous_pts:
            current_median = ptset.get_median()
            if current_median >= 3 and current_median <= 3.28:
                continue
            else:
                #self.publish_collision_scan(ptset, len(laser_scan.ranges), laser_scan)
                self.publish_picked_heading_scan(ptset.get_angle(),
                        current_median, len(laser_scan.ranges), laser_scan)
                rospy.logdebug("Picking angle %f median %f", ptset.get_angle(), current_median)
                return ptset.get_angle()

        return None

    def publish_picked_heading_scan(self, avg_angle, median_dist, num_pts, laser_scan):
        scan = LaserScan()
        
        scan.ranges = [0.0] * num_pts
        scan.intensities = [0.0] * num_pts
        angle_per_pt = (2 * 3.142) / len(scan.ranges)
        current_angle = 0
        for i in range(len(scan.ranges)):
            if current_angle >= (avg_angle - 0.25) and current_angle <= (avg_angle + 0.25):
                scan.ranges[i] = median_dist + (i * 0.01)
                scan.intensities[i] = 10.0
            
            current_angle += angle_per_pt

        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = laser_scan.header.frame_id
        scan.range_min = laser_scan.range_min
        scan.range_max = laser_scan.range_max

        self.pub.publish(scan)


    def publish_collision_scan(self, ptset, num_pts, laser_scan):
        scan = LaserScan()
        
        scan.ranges = [0.0] * num_pts
        scan.intensities = [0.0] * num_pts
        for i in range(len(ptset.pts)):
            scan.ranges[ptset.start_pt_num+i] = ptset.pts[i]
            #scan.intensities[ptset.start_pt_num+i] = 10.0

        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = laser_scan.header.frame_id
        scan.range_min = laser_scan.range_min
        scan.range_max = laser_scan.range_max

        self.pub.publish(scan)

if __name__ == "__main__":
      rospy.init_node('laser_scan_processor_node')
 
      scanner = LaserScanProcessor()
      rospy.Subscriber("scan", LaserScan, scanner.process_laser_scan, queue_size=1)
      rospy.spin()
