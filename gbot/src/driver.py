#!/usr/bin/env python
import random
import rospy
import time
from track_imu import TrackImu
from track_vertical_pose import TrackVerticalPose
from track_encoders import TrackEncoders
from robot_state import RobotState, CommandSource, RobotStateTracker
from speed_tracker import SpeedTracker
from std_msgs.msg import String, Int16
from gbot.msg import RobotCmd
from track_pose import TrackPose

class Driver:
    LEFT_TURN_TIME_PER_RADIAN = 0.26
    RIGHT_TURN_TIME_PER_RADIAN = 0.28

    def __init__(self):
        self.speed_tracker = SpeedTracker()
        self.state_tracker = RobotStateTracker()
        self.track_encoders = TrackEncoders(self)
        self.vertical_pose_tracker = TrackVerticalPose(self)
        self.track_pose = TrackPose(self)
        #self.track_imu = TrackImu()
        self.emergency_stop = False

        self.pub = rospy.Publisher("robot_commands", RobotCmd, queue_size=1)
        self.lmotor_pub = rospy.Publisher("lmotor", Int16, queue_size=1)
        self.rmotor_pub = rospy.Publisher("rmotor", Int16, queue_size=1)

    def open(self):
        pass

    def spin(self):
        pass

    def forward(self):
        self.execute_cmd(RobotState.FORWARD)

    def turn_left(self):
        self.turn_angle(1.57 + 3.142)

    def turn_right(self):
        self.turn_angle(1.57)

    def random_turn(self):
        rand = random.random()
        if rand <= 0.5:
            self.turn_left()
        else:
            self.turn_right()

    def stop_and_turn_robot(self, angle, left_obstacle, right_obstacle):
        ## Straight ahead is 0 radians, increasing clockwise to 2 pi
        if angle <= 3.142:
            if right_obstacle == False:
                # Turn right
                self.execute_cmd(RobotState.RIGHT)
                self.track_angular_change(RobotState.RIGHT, angle)
                self.execute_cmd(RobotState.STOP)
            else:
                # Turn right via left
                self.execute_cmd(RobotState.LEFT)
                self.track_angular_change(RobotState.LEFT, 3.142 + (3.142 - angle))
                self.execute_cmd(RobotState.STOP)
        else:
            if left_obstacle == False:
                # Turn left
                complement = 3.142 - (angle - 3.142)
                self.execute_cmd(RobotState.LEFT)
                self.track_angular_change(RobotState.LEFT, complement) # complement of angle
                self.execute_cmd(RobotState.STOP)
            else:
                # Turn left via right
                self.execute_cmd(RobotState.RIGHT)
                self.track_angular_change(RobotState.RIGHT, angle) # This is a clockwise turn - go through full angle
                self.execute_cmd(RobotState.STOP)

    def compute_steering_ratio(self, angle):
        ratio = angle / 1.85
        return 1 - ratio

    def steer_robot(self, angle):
        if angle <= 3.142:
            # Turn right
            steering_ratio = self.compute_steering_ratio(angle)
            self.execute_cmd(RobotState.STEER_RIGHT, steering_ratio)
        else:
            # Turn left
            complement = 3.142 - (angle - 3.142)
            steering_ratio = self.compute_steering_ratio(complement)
            self.execute_cmd(RobotState.STEER_LEFT, steering_ratio)

    def turn_angle(self, angle, left_obstacle=False, right_obstacle=False):
        if self.state_tracker.get_last_state() == RobotState.STOP:
            rospy.loginfo("Stopping & turning robot angle %f left_obstacle %s right_obstacle %s",
                    angle, left_obstacle, right_obstacle)
            self.stop_and_turn_robot(angle, left_obstacle, right_obstacle)
        else:
            rospy.loginfo("Steering robot angle %f left_obstacle %s right_obstacle %s",
                    angle, left_obstacle, right_obstacle)
            self.steer_robot(angle)

    def reverse(self, delay=1.0):
        self.execute_cmd(RobotState.REVERSE)
        time.sleep(delay)
        self.execute_cmd(RobotState.STOP)

    def stop(self, track_state=True):
        self.execute_cmd(RobotState.STOP, track_state)

    def do_emergency_stop(self):
        self.stop()
        self.emergency_stop = True

    def track_state(self, cmd):
        self.state_tracker.add(cmd)
        
        data = RobotCmd()
        data.header.stamp = rospy.Time.now()
        data.cmd = cmd
        self.pub.publish(data)

    def execute_cmd(self, cmd, steering_ratio=1.0, track_state=True):
        rospy.loginfo("Executing %s fwd speed: %d steering ratio: %f", cmd, self.speed_tracker.forward_speed,
                steering_ratio)
        if track_state:
            self.track_state(cmd)

        if self.emergency_stop and cmd != RobotState.STOP:
             rospy.logerr("Emergency stop. Ignoring %s command", cmd)
             return
        
        lmotor_msg = Int16()
        rmotor_msg = Int16()
        if cmd == RobotState.FORWARD:
            lmotor_msg.data = self.speed_tracker.forward_speed
            rmotor_msg.data = self.speed_tracker.forward_speed
        elif cmd == RobotState.REVERSE:
            lmotor_msg.data = -self.speed_tracker.reverse_speed
            rmotor_msg.data = -self.speed_tracker.reverse_speed
        elif cmd == RobotState.LEFT:
            lmotor_msg.data = -self.speed_tracker.forward_speed * 0.9
            rmotor_msg.data = self.speed_tracker.forward_speed * 0.9
        elif cmd == RobotState.RIGHT:
            lmotor_msg.data = self.speed_tracker.forward_speed * 0.9
            rmotor_msg.data = -self.speed_tracker.forward_speed * 0.9
        elif cmd == RobotState.STEER_LEFT:
            lmotor_msg.data = int(self.speed_tracker.forward_speed * steering_ratio)
            rmotor_msg.data = self.speed_tracker.forward_speed
        elif cmd == RobotState.STEER_RIGHT:
            lmotor_msg.data = self.speed_tracker.forward_speed
            rmotor_msg.data = int(self.speed_tracker.forward_speed * steering_ratio)
        elif cmd == RobotState.STOP:
            lmotor_msg.data = 0
            rmotor_msg.data = 0
            self.speed_tracker.reset_forward_speed()

        self.lmotor_pub.publish(lmotor_msg)
        self.rmotor_pub.publish(rmotor_msg)

    def track_angular_change(self, state, turn_angle):
#        if self.track_imu.should_use_imu():
#            rospy.loginfo("Tracking IMU change for turns")
#            self.track_imu_for_angular_change(turn_angle)
            rospy.loginfo("Tracking pose change for turns")
            self.track_for_angular_change(state, turn_angle, self.track_pose)
#
#        else:
#            rospy.logwarn("IMU data not available - falling back to time based turns")
#            self.track_time_for_angular_change(state, turn_angle)

    def track_time_for_angular_change(self, state, turn_angle):
        sleep_time = self.get_turn_time_for_angle(state, turn_angle)
        time.sleep(sleep_time)

    def get_turn_time_for_angle(self, state, turn_angle):
        sleep_time = 0.0
        if state == RobotState.LEFT:
            sleep_time = Driver.LEFT_TURN_TIME_PER_RADIAN * turn_angle
        else:
            sleep_time = Driver.RIGHT_TURN_TIME_PER_RADIAN * turn_angle
        sleep_time = 0.04 if sleep_time < 0.04 else sleep_time
        sleep_time = 0.1 if sleep_time > 0.1 else sleep_time

        return sleep_time

    def track_for_angular_change(self, state, turn_angle, tracker):
        start_time = time.time()

        tracker.start_tracking()
        #for i in range(0, int(turn_time / 0.1)):
        corrections = 0
        while(True):
            sleep_time = self.get_turn_time_for_angle(state, abs(turn_angle - tracker.get_current_angle()))
            self.execute_cmd(state)
            time.sleep(sleep_time)
            self.stop()
            time.sleep(0.2)
            angular_change = abs(tracker.get_current_angle())
            rospy.loginfo("Angular change %f (want %f)", angular_change, turn_angle)
            
            if angular_change >= turn_angle:
                if angular_change <= (turn_angle + 0.05):
                    rospy.loginfo("Turn complete. Wanted %f angular change %f", turn_angle, angular_change)
                    break
                else:
                    # Ugh over turned - correct yourself
                    state = RobotState.LEFT if state == RobotState.RIGHT else RobotState.RIGHT
                    rospy.loginfo("Turned too much")
                    corrections += 1

            if corrections > 3 or (time.time() - start_time) > 4:
                break

        tracker.stop_tracking()

if __name__== "__main__":
    from dimension_driver import DimensionDriver
    
    rospy.init_node("driver")
    motor_driver = DimensionDriver(128, '/dev/ttyUSB0')
    driver = Driver()

    try:
        motor_driver.open()
        driver.open()
        driver.turn_left()
        time.sleep(1)
        driver.turn_right()
        time.sleep(1)
        driver.forward()
        time.sleep(1)
        driver.reverse()
        time.sleep(1)
        driver.turn_angle(0.7855)
        time.sleep(1)
        driver.turn_angle(5.4985)
        time.sleep(1)
        driver.turn_angle(1.571, False, True)
        time.sleep(1)
        driver.turn_angle(4.713, True, False)
        time.sleep(1)

        driver.stop()
    finally:
        motor_driver.close()
        pass
