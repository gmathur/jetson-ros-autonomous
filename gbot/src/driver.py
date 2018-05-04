#!/usr/bin/env python
import rospy
import time
from encoder import RobotEncoderController
from track_imu import TrackImu
from robot_state import RobotState, CommandSource, RobotStateTracker
from speed_tracker import SpeedTracker

class Driver:
    TURN_TIME_PER_RADIAN = 0.318

    def __init__(self, dimension_driver):
        self.driver = dimension_driver
        self.speed_tracker = SpeedTracker()
        self.state_tracker = RobotStateTracker()
        self.encoder_controller = RobotEncoderController()
        self.track_imu = TrackImu()

    def open(self):
        self.encoder_controller.stop()
        self.driver.open()

    def spin(self):
        self.encoder_controller.spin()

    def forward(self):
        self.execute_cmd(RobotState.FORWARD)

    def turn_left(self):
        self.turn_angle(1.57 + 3.142)

    def turn_right(self):
        self.turn_angle(1.57)

    def turn_angle(self, angle):
        if angle <= 3.142:
            # Turn right
            self.execute_cmd(RobotState.RIGHT)
            self.track_angular_change(turn_angle=angle)
            self.execute_cmd(RobotState.STOP)
        else:
            # Turn left
            self.execute_cmd(RobotState.LEFT)
            self.track_angular_change(turn_angle=angle - 3.142)
            self.execute_cmd(RobotState.STOP)

    def reverse(self):
        self.execute_cmd(RobotState.REVERSE)
        time.sleep(1)
        self.execute_cmd(RobotState.STOP)

    def stop(self):
        self.execute_cmd(RobotState.STOP)

    def execute_cmd(self, cmd):
        rospy.loginfo("Executing %s fwd speed: %d" % (cmd, self.speed_tracker.forward_speed))
        self.state_tracker.add(cmd)
        
        self.encoder_controller.set_state(cmd)
        
        if cmd == RobotState.FORWARD:
            self.driver.drive_forward(self.speed_tracker.forward_speed)
        elif cmd == RobotState.REVERSE:
            self.driver.drive_backward(self.speed_tracker.reverse_speed)
        elif cmd == RobotState.LEFT:
            self.driver.turn_left(self.speed_tracker.turn_speed)
        elif cmd == RobotState.RIGHT:
            self.driver.turn_right(self.speed_tracker.turn_speed)
        elif cmd == RobotState.STOP:
            self.driver.stop()
            self.speed_tracker.reset_forward_speed()

    def track_angular_change(self, turn_angle=1.57):
        if self.track_imu.should_use_imu():
            rospy.loginfo("Tracking IMU change for turns")
            self.track_imu_for_angular_change(turn_angle)
        else:
            rospy.logwarn("IMU data not available - falling back to time based turns")
            self.track_time_for_angular_change(turn_angle)

    def track_time_for_angular_change(self, turn_angle=1.57):
        sleep_time = Driver.TURN_TIME_PER_RADIAN * turn_angle
        sleep_time = 0.1 if sleep_time < 0.1 else sleep_time
        time.sleep(sleep_time)

    def track_imu_for_angular_change(self, turn_angle=1.57):
        start_time = time.time()

        self.track_imu.reset()
        #for i in range(0, int(turn_time / 0.1)):
        while(True):
            time.sleep(0.01)
            angular_change = abs(self.track_imu.get_angular_change())
            rospy.logdebug("Angular change %f (want %f)", angular_change, turn_angle)
            
            if angular_change >= turn_angle:
                rospy.loginfo("Turn complete. Wanted %f angular change %f", turn_angle, angular_change)
                return False

            if (time.time() - start_time) > 2:
                return True

