#!/usr/bin/env python
import rospy
import time
from track_imu import TrackImu
from track_encoders import TrackEncoders
from robot_state import RobotState, CommandSource, RobotStateTracker
from speed_tracker import SpeedTracker
from std_msgs.msg import String, Int16
from gbot.msg import RobotCmd

class EncoderMotionTracker:
    def __init__(self):
        rospy.Subscriber("lwheel", Int16, self.lwheel_callback, queue_size=1)
        rospy.Subscriber("rwheel", Int16, self.rwheel_callback, queue_size=1)
        self.lwheel_start = None
        self.rwheel_start = None
        self.lwheel_last = None
        self.rwheel_last = None

        self.reset_lwheel = True
        self.reset_rwheel = True

    def lwheel_callback(self, data):
        if self.reset_lwheel:
            self.reset_lwheel = False
            self.lwheel_start = data.data

        self.lwheel_last = data.data

    def rwheel_callback(self, data):
        if self.reset_rwheel:
            self.reset_rwheel = False
            self.rwheel_start = data.data
        
        self.rwheel_last = data.data

    def lwheel_delta(self):
        return self.lwheel_last - self.lwheel_start

    def rwheel_delta(self):
        return self.rwheel_last - self.rwheel_start

    def did_robot_move(self):
        movement = self.lwheel_delta() + self.rwheel_delta()
        rospy.loginfo("Robot moved %d encoder counts", movement)

        return True if movement > 1 else False

class Driver:
    TURN_TIME_PER_RADIAN = 0.318

    def __init__(self, dimension_driver):
        self.driver = dimension_driver
        self.speed_tracker = SpeedTracker()
        self.state_tracker = RobotStateTracker()
        self.track_imu = TrackImu(self)
        self.track_encoders = TrackEncoders(self)
        self.motion_tracker = EncoderMotionTracker()
        self.emergency_stop = False

        self.pub = rospy.Publisher("robot_commands", RobotCmd, queue_size=1)

    def open(self):
        self.driver.open()

    def spin(self):
        pass

    def forward(self):
        self.execute_cmd(RobotState.FORWARD)

    def turn_left(self):
        self.turn_angle(1.57 + 3.142)

    def turn_right(self):
        self.turn_angle(1.57)

    def turn_angle(self, angle, left_obstacle=False, right_obstacle=False):
        ## Straight ahead is 0 radians, increasing clockwise to 2 pi
        if angle <= 3.142:
            if right_obstacle is False:
                # Turn right
                self.execute_cmd(RobotState.RIGHT)
                self.track_angular_change(turn_angle=angle)
                self.execute_cmd(RobotState.STOP)
            else:
                # Turn right via left
                self.execute_cmd(RobotState.LEFT)
                self.track_angular_change(turn_angle=3.142 + (3.142 - angle))
                self.execute_cmd(RobotState.STOP)

        else:
            if left_obstacle is False:
                # Turn left
                self.execute_cmd(RobotState.LEFT)
                self.track_angular_change(turn_angle=3.142 - (angle - 3.142)) # complement of angle
                self.execute_cmd(RobotState.STOP)
            else:
                # Turn left via right
                self.execute_cmd(RobotState.RIGHT)
                self.track_angular_change(turn_angle=angle) # This is a clockwise turn - go through full angle
                self.execute_cmd(RobotState.STOP)

    def reverse(self):
        self.execute_cmd(RobotState.REVERSE)
        time.sleep(1)
        self.execute_cmd(RobotState.STOP)

    def stop(self):
        self.execute_cmd(RobotState.STOP)

    def do_emergency_stop(self):
        self.stop()
        self.emergency_stop = True

    def execute_cmd(self, cmd):
        rospy.loginfo("Executing %s fwd speed: %d" % (cmd, self.speed_tracker.forward_speed))
        self.state_tracker.add(cmd)
        
        data = RobotCmd()
        data.header.stamp = rospy.Time.now()
        data.cmd = cmd
        self.pub.publish(data)

        if self.emergency_stop and cmd != RobotState.STOP:
             rospy.logerr("Emergency stop. Ignoring %s command", cmd)
             return
        
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
#        if self.track_imu.should_use_imu():
#            rospy.loginfo("Tracking IMU change for turns")
#            self.track_imu_for_angular_change(turn_angle)
#        else:
#            rospy.logwarn("IMU data not available - falling back to time based turns")
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
            time.sleep(0.001)
            angular_change = abs(self.track_imu.get_angular_change())
            rospy.logdebug("Angular change %f (want %f)", angular_change, turn_angle)
            
            if angular_change >= turn_angle:
                rospy.loginfo("Turn complete. Wanted %f angular change %f", turn_angle, angular_change)
                return False

            if (time.time() - start_time) > 2:
                return True

if __name__== "__main__":
    from dimension_driver import DimensionDriver
    
    rospy.init_node("driver")
    motor_driver = DimensionDriver(128, '/dev/ttyUSB0')
    driver = Driver(motor_driver)
    driver.open()

    try:
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
