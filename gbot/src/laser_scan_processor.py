#!/usr/bin/env python
import rospy
import random
import threading
import time
from robot_state import RobotState
from gbot.msg import Proximity
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy

MARGIN = 0.1
MIN_DIST = 0.3
FRONT_MIN_DIST = 0.3

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
        return (((self.start_angle + self.end_angle) / 2.0) + 3.142) % (2 * 3.142)

class LaserScanProcessor:
    def __init__(self):
        self.obstacles_pub = rospy.Publisher('scan_obstacles', LaserScan, queue_size=10)
        self.front_obstacles_pub = rospy.Publisher('scan_front_obstacles', LaserScan, queue_size=10)
        self.heading_pub = rospy.Publisher('scan_heading', LaserScan, queue_size=10)
        
        rospy.Subscriber('robot_commands', String, self.robot_cmds_callback, queue_size=10)
        self.last_robot_state = RobotState.FORWARD
        self.front_obstacle_scan_processing = False

    def robot_cmds_callback(self, data):
        self.last_robot_state = data.data

    def check_for_front_obstacles(self, laser_scan):
        if not self.last_robot_state in [RobotState.FORWARD, RobotState.STOP]:
            # Dont process laser scans while turning
            return None

        if self.front_obstacle_scan_processing:
            return None

        self.front_obstacle_scan_processing = True

        # Calculate range of laser scan pts to evaluate
        min_range = int(((3.142 / 6) * 5) / laser_scan.angle_increment)
        max_range = int(((3.142 / 6) * 7) / laser_scan.angle_increment)

        num_matches = 0
        ptset = ContiguousScanPoints()
        for i in range(min_range, max_range):
            current_angle = i * laser_scan.angle_increment
            pt = laser_scan.ranges[i]
            if pt > 0 and pt < FRONT_MIN_DIST:
                num_matches += 1
                ptset.add(1.0, current_angle, i)

        if num_matches > 3:
            rospy.loginfo("Front obstacle encountered")
            self.publish_obstacles_scan([ptset], len(laser_scan.ranges), laser_scan,
                    self.front_obstacles_pub)
            self.front_obstacle_scan_processing = False
            return True
        else:
            rospy.loginfo("No obstacles encountered")
            self.front_obstacle_scan_processing = False
            return False

    def process_laser_scan(self, laser_scan):
        if not self.last_robot_state in [RobotState.FORWARD, RobotState.STOP]:
            rospy.loginfo("Ignoring laser scan request as robot state is %s", self.last_robot_state)
            return None

        rospy.loginfo("Processing laser scan to find direction")
        contiguous_pts = [ContiguousScanPoints()]

        # Find largest distance in scan
        current_angle = 0
        
        # Find all contiguous points
        for i in range(len(laser_scan.ranges)):
            pt = laser_scan.ranges[i]
            if pt < laser_scan.range_min or pt > laser_scan.range_max:
                rospy.logdebug("Skipping laser pt at %f", current_angle)
                continue

            pt = laser_scan.range_max if pt == 0 else pt
            if not contiguous_pts[len(contiguous_pts)-1].add(pt, current_angle, i):
                new_contiguous_pts = ContiguousScanPoints()
                new_contiguous_pts.add(pt, current_angle, i)
                contiguous_pts.append(new_contiguous_pts)
            
            current_angle += laser_scan.angle_increment

        # Clean up the contiguous pts
        contiguous_pts = [ptset for ptset in contiguous_pts \
                                if ptset.num() > 10 and ptset.get_median() >= MIN_DIST]

        self.publish_obstacles_scan(contiguous_pts, len(laser_scan.ranges), laser_scan,
                self.obstacles_pub)

        # Now look for largest pts
        contiguous_pts = sorted(contiguous_pts, key=lambda item: item.get_median(), reverse=True)
        
        # Pick the largest one that is not right behind
        for ptset in contiguous_pts:
            current_median = ptset.get_median()
            if current_median >= 3 and current_median <= 3.28:
                continue
            else:
                self.publish_picked_heading_scan(ptset.get_angle(), len(laser_scan.ranges), laser_scan)
                rospy.loginfo("Picking heading %f with median dist %f", ptset.get_angle(), current_median)
                return ptset.get_angle()

        rospy.loginfo("No good heading except back")
        return -1

    def publish_picked_heading_scan(self, avg_angle, num_pts, laser_scan):
        scan = LaserScan()
        
        scan.ranges = [0.0] * num_pts
        scan.intensities = [0.0] * num_pts
        current_angle = 0
        for i in range(len(scan.ranges)):
            if current_angle >= (avg_angle - 0.25) and current_angle <= (avg_angle + 0.25):
                scan.ranges[i] = 1.0 + (i * 0.01)
                scan.intensities[i] = 10.0
            
            current_angle += laser_scan.angle_increment

        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = laser_scan.header.frame_id
        scan.range_min = laser_scan.range_min
        scan.range_max = laser_scan.range_max
        scan.angle_min = laser_scan.angle_min
        scan.angle_max = laser_scan.angle_max
        scan.angle_increment = laser_scan.angle_increment

        self.heading_pub.publish(scan)


    def publish_obstacles_scan(self, contiguous_pts, num_pts, laser_scan,
            pub):
        scan = LaserScan()
        
        scan.ranges = [0.0] * num_pts
        scan.intensities = [0.0] * num_pts
        for ptset in contiguous_pts:
            for i in range(len(ptset.pts)):
                scan.ranges[ptset.start_pt_num + i] = ptset.pts[i]
                scan.intensities[ptset.start_pt_num+i] = 10.0

        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = laser_scan.header.frame_id
        scan.range_min = laser_scan.range_min
        scan.range_max = laser_scan.range_max
        scan.angle_min = laser_scan.angle_min
        scan.angle_max = laser_scan.angle_max
        scan.angle_increment = laser_scan.angle_increment

        pub.publish(scan)

if __name__ == "__main__":
      rospy.init_node('laser_scan_processor_node')
 
      scanner = LaserScanProcessor()
      rospy.Subscriber("scan", LaserScan, scanner.process_laser_scan, queue_size=1)
      rospy.Subscriber("scan", LaserScan, scanner.check_for_front_obstacles, queue_size=1)
      rospy.loginfo("Laser scan processor init")
      rospy.spin()
