#!/usr/bin/env python
import rospy
import time
from encoder import RobotEncoderController
from track_imu import TrackImu
from robot_state import RobotState, CommandSource

class Driver:
    def __init__(self, dimension_driver):
        self.driver = dimension_driver
        self.encoder_controller = RobotEncoderController()
        self.track_imu = TrackImu()

    def open(self):
        self.encoder_controller.stop()
        self.driver.open()

    def spin(self):
        self.encoder_controller.spin()

    def forward(self):
        self.execute_cmd(RobotState.FORWARD)

    def turn_left(self, turn_time = TURN_TIME):
        self.execute_cmd(RobotState.LEFT)
        self.track_angular_imu_for_time(turn_time)
        self.execute_cmd(RobotState.STOP)

    def turn_right(self, turn_time = TURN_TIME):
        self.execute_cmd(RobotState.RIGHT)
        self.track_angular_imu_for_time(turn_time)
        self.execute_cmd(RobotState.STOP)

    def reverse(self):
        self.execute_cmd(RobotState.REVERSE)
        time.sleep(1)
        self.execute_cmd(RobotState.STOP)

    def stop(self):
        self.execute_cmd(RobotState.STOP)

    def execute_cmd(self, cmd):
        rospy.loginfo("Executing %s fwd speed: %d" % (cmd, self.forward_speed))
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

    def track_angular_imu_for_time(self, turn_time, turn_angle=1.57):
        start_time = time.time()

        self.track_imu.reset()
        #for i in range(0, int(turn_time / 0.1)):
        while(True):
            time.sleep(0.2)
            angular_change = abs(self.track_imu.get_angular_change())
            rospy.loginfo("Angular change %f" % (angular_change))
            if angular_change < 0.1:
                # Turn isnt happening for whatever reason
                rospy.loginfo("Aborting turn as IMU isnt changing")
                return True
            
            if angular_change > turn_angle:
                return False

            if (time.time() - start_time) > 2:
                return True

