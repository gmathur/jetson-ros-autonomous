#!/usr/bin/env python
import rospy
import random
import threading
import time
import math
from robot_state import RobotState
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from gbot.msg import RobotCmd
import numpy

MARGIN = 0.15
MIN_DIST = 0.5
STEER_MIN_DIST = 1.0
FRONT_OBSTACLE_THRESHOLD = 0.30
FRONT_STEER_THRESHOLD = 0.7
SIDE_OBSTACLE_THRESHOLD = 0.16
ROBOT_WIDTH = 0.25

class ContiguousScanPoints:

    def __init__(self):
        self.pts = []
        self.start_angle = -1
        self.end_angle = -1
        self.median = None
        self.start_pt_num = -1

    def add(self, pt, angle, num):
        if len(self.pts) == 0:
            if pt < MIN_DIST:
                return True

            self.pts.append(pt)
            self.start_angle = angle
            self.start_pt_num = num
            return True
        
        last_pt = self.pts[len(self.pts)-1]
        if pt >= MIN_DIST and \
           pt <= last_pt + (last_pt * MARGIN) and \
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
        return (3.142 - ((self.start_angle + self.end_angle) / 2.0)) % (2 * 3.142)

    def get_gap_size(self):
        if len(self.pts) < 2:
            return 0

        a = self.pts[0]
        b = self.pts[len(self.pts)-1]
        return math.sqrt(
                pow(a, 2) + pow(b, 2) - 2 * a * b * math.cos(abs(self.end_angle - self.start_angle))
            )

class LaserScanProcessor:
    def __init__(self):
        self.options_pub = rospy.Publisher('scan_options', LaserScan, queue_size=10)
        self.obstacles_pub = rospy.Publisher('scan_obstacles', LaserScan, queue_size=10)
        self.rotate_heading_pub = rospy.Publisher('rotate_heading', LaserScan, queue_size=10)
        self.steer_heading_pub = rospy.Publisher('steer_heading', LaserScan, queue_size=10)
        
        rospy.Subscriber('robot_commands', RobotCmd, self.robot_cmds_callback, queue_size=10)
        self.last_robot_state = None
        self.front_obstacle_scan_processing = False

        rospy.loginfo("Laser scan processor init ok")

    def robot_cmds_callback(self, data):
        self.last_robot_state = data

    def steer(self, laser_scan):
        if self.front_obstacle_scan_processing:
            return None, -1

        self.front_obstacle_scan_processing = True

        # Calculate range of laser scan pts to evaluate
        min_range = int(((3.142 / 6) * 5) / laser_scan.angle_increment)
        max_range = int(((3.142 / 6) * 7) / laser_scan.angle_increment)

        obstacles_present, min_dist = self.is_obstacle_present(laser_scan, min_range, max_range,
                FRONT_OBSTACLE_THRESHOLD, 6)

        if obstacles_present:
            self.front_obstacle_scan_processing = False    
            rospy.loginfo("Obstacles present - not processing further")
            return obstacles_present, None

        if min_dist > FRONT_STEER_THRESHOLD:
            self.front_obstacle_scan_processing = False    
            rospy.loginfo("Front min dist %f - no need to steer", min_dist)
            return False, 0

        rospy.loginfo("Front min dist %f - computing where to steer", min_dist)

        # No immediate obstacles - steer
        min_range = int(((3.142 / 2) * 1) / laser_scan.angle_increment)
        max_range = int(((3.142 / 2) * 3) / laser_scan.angle_increment)

        contiguous_pts = self.compute_possible_options(laser_scan, 1.571, min_range, max_range, STEER_MIN_DIST)

        if len(contiguous_pts) == 0:
            self.front_obstacle_scan_processing = False
            rospy.loginfo("No options found - returning as obstacles")
            return True, -1

        self.publish_options_scan(contiguous_pts, len(laser_scan.ranges), laser_scan,
                self.options_pub)

        # Sprt by closeness to center
        angles_from_zero = [(item, item.get_angle()) if item.get_angle() <= 3.142 else (item, abs(item.get_angle() - 6.284)) for item in contiguous_pts]
        contiguous_pts = sorted(angles_from_zero, key=lambda item: item[1])
        ptset = contiguous_pts[0][0]

        self.publish_options_scan([ptset], len(laser_scan.ranges), laser_scan,
                 self.steer_heading_pub)
        self.publish_picked_heading_scan(ptset.get_angle(), len(laser_scan.ranges), laser_scan,
                self.steer_heading_pub)
        
        rospy.loginfo("Steer towards angle %f", ptset.get_angle())
        self.front_obstacle_scan_processing = False
        return False, ptset.get_angle()

    def is_obstacle_present(self, laser_scan, min_range, max_range, dist_threshold, pt_threshold=3):
        num_matches = 0
        min_dist = laser_scan.range_max
        ptset = ContiguousScanPoints()
        for i in range(min_range, max_range):
            current_angle = i * laser_scan.angle_increment
            pt = laser_scan.ranges[i]
            if pt > 0 and pt < dist_threshold:
                num_matches += 1
                ptset.add(1.0, current_angle, i)
            
            if pt > 0.18 and pt < min_dist:
                min_dist = pt

        if num_matches > pt_threshold:
            rospy.loginfo("Obstacle encountered min angle %f max angle %f matches %d:", min_range, max_range, num_matches)
            self.publish_options_scan([ptset], len(laser_scan.ranges), laser_scan,
                 self.obstacles_pub)
            return True, min_dist
        else:
            return False, min_dist
 
    def check_for_side_obstacles(self, laser_scan):
        # Calculate range of laser scan pts to evaluate
        left_min_range = int(((3.142 / 6) * 7) / laser_scan.angle_increment)
        left_max_range = int(((3.142 / 6) * 11) / laser_scan.angle_increment)

        left_obstacle_present, _ = self.is_obstacle_present(laser_scan, left_min_range, left_max_range, SIDE_OBSTACLE_THRESHOLD, 3)

        right_min_range = int(((3.142 / 6) * 1) / laser_scan.angle_increment)
        right_max_range = int(((3.142 / 6) * 5) / laser_scan.angle_increment)

        right_obstacle_present, _ = self.is_obstacle_present(laser_scan, right_min_range, right_max_range, SIDE_OBSTACLE_THRESHOLD, 3)

        return left_obstacle_present, right_obstacle_present

    def compute_possible_options(self, laser_scan, start_angle, min_range, max_range, min_dist):
        contiguous_pts = [ContiguousScanPoints()]

         # Find largest distance in scan
        current_angle = start_angle
        
        # Find all contiguous points
        for i in range(min_range, max_range):
            pt = laser_scan.ranges[i]
            pt = laser_scan.ranges[i-1] if pt == 0.0 else pt

            if not contiguous_pts[len(contiguous_pts)-1].add(pt, current_angle, i):
                new_contiguous_pts = ContiguousScanPoints()
                new_contiguous_pts.add(pt, current_angle, i)
                contiguous_pts.append(new_contiguous_pts)
            
            current_angle += laser_scan.angle_increment

        # Clean up the contiguous pts
        contiguous_pts = [ptset for ptset in contiguous_pts \
                                if \
                                    ptset.get_gap_size() >= ROBOT_WIDTH and \
                                    ptset.get_median() >= min_dist]

        return contiguous_pts

    def pick_heading(self, laser_scan):
        if self.last_robot_state and \
                laser_scan.header.stamp > self.last_robot_state.header.stamp:
            rospy.logdebug("Ignoring laser scan request as robot state is %s", self.last_robot_state)
            return None, None, None

        contiguous_pts = self.compute_possible_options(laser_scan, 0, 0, len(laser_scan.ranges), MIN_DIST)
       
        #self.publish_options_scan(contiguous_pts, len(laser_scan.ranges), laser_scan,
        #        self.options_pub)

        # Now look for longest dist
        contiguous_pts = sorted(contiguous_pts, key=lambda item: item.get_median(), reverse=True)
        
        # Pick the largest one that is not right behind
        left_obstacle, right_obstacle = self.check_for_side_obstacles(laser_scan)
        for ptset in contiguous_pts:
            current_median = ptset.get_median()
            if current_median >= 3 and current_median <= 3.28:
                continue
            else:
                #self.publish_picked_heading_scan(ptset.get_angle(), len(laser_scan.ranges), laser_scan,
                #        self.rotate_heading_pub)
                self.publish_options_scan([ptset], len(laser_scan.ranges), laser_scan,
                    self.rotate_heading_pub)
                rospy.loginfo("Picking heading %f with median dist %f left_obstacle %s right_obstacle %s", 
                        ptset.get_angle(), current_median, left_obstacle, right_obstacle)
                return ptset.get_angle(), left_obstacle, right_obstacle

        rospy.loginfo("No good heading except reverse left_obstacle %s right_obstacle %s", left_obstacle, right_obstacle)
        return -1, left_obstacle, right_obstacle

    def publish_picked_heading_scan(self, avg_angle, num_pts, laser_scan, pub):
        scan = LaserScan()
        
        scan.ranges = [0.0] * num_pts
        scan.intensities = [0.0] * num_pts
        current_angle = 0
        avg_angle = (3.142 - avg_angle) % (2 * 3.142)
        for i in range(len(scan.ranges)):
            if current_angle >= (avg_angle - 0.1) and current_angle <= (avg_angle + 0.1):
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

        pub.publish(scan)


    def publish_options_scan(self, contiguous_pts, num_pts, laser_scan,
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
      rospy.Subscriber("scan", LaserScan, scanner.pick_heading, queue_size=1)
      rospy.Subscriber("scan", LaserScan, scanner.steer, queue_size=1)
      rospy.loginfo("Laser scan processor init")
      rospy.spin()
