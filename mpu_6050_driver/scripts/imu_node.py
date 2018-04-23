#!/usr/bin/env python

import sys
import time
import smbus
import struct
import rospy
import numpy as np
from kalman import KalmanFilter
from sensor_msgs.msg import Temperature, Imu
from tf.transformations import quaternion_about_axis
from mpu_6050_driver.registers import PWR_MGMT_1, ACCEL_XOUT_H, ACCEL_YOUT_H, ACCEL_ZOUT_H, TEMP_H,\
    GYRO_XOUT_H, GYRO_YOUT_H, GYRO_ZOUT_H

ADDR = None
bus = None
IMU_FRAME = None

process_variance = 1e-4 # original was 1e-3
kalman_GyroX = KalmanFilter(process_variance, 0.2351505886 ** 2)
kalman_GyroY = KalmanFilter(process_variance, 0.1027677148 ** 2)
kalman_GyroZ = KalmanFilter(process_variance, 0.09293135685 ** 2)
kalman_AccelX = KalmanFilter(process_variance, 0.004643714836 ** 2)
kalman_AccelY = KalmanFilter(process_variance, 0.003318472413 ** 2)
kalman_AccelZ = KalmanFilter(process_variance, 0.005926761194 ** 2)

# read_word and read_word_2c from http://blog.bitify.co.uk/2013/11/reading-data-from-mpu-6050-on-raspberry.html
def read_word(adr):
    high = bus.read_byte_data(ADDR, adr)
    low = bus.read_byte_data(ADDR, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def publish_temp(timer_event):
    try:
        temp_msg = Temperature()
        temp_msg.header.frame_id = IMU_FRAME
    	temp_msg.temperature = read_word_2c(TEMP_H)/340.0 + 36.53
    	temp_msg.header.stamp = rospy.Time.now()
    	temp_pub.publish(temp_msg)
    except:
        print("Unexpected error", sys.exc_info()[0])

def get_orientation(accel):
    ref = np.array([0, 0, 1])
    acceln = accel / np.linalg.norm(accel)
    axis = np.cross(acceln, ref)
    angle = np.arccos(np.dot(acceln, ref))
    orientation = quaternion_about_axis(angle, axis)

    return orientation

def publish_imu(timer_event):
    try:
	imu_msg = Imu()
	imu_msg.header.frame_id = IMU_FRAME

        raw_imu_msg = Imu()
        raw_imu_msg.header.frame_id = IMU_FRAME

	# Read the acceleration vals
	raw_accel_x = read_word_2c(ACCEL_XOUT_H) / 16384.0
	raw_accel_y = read_word_2c(ACCEL_YOUT_H) / 16384.0
	raw_accel_z = read_word_2c(ACCEL_ZOUT_H) / 16384.0

	kalman_AccelX.noisy_measurement(raw_accel_x)
	kalman_AccelY.noisy_measurement(raw_accel_y)
	kalman_AccelZ.noisy_measurement(raw_accel_z)

	accel_x = kalman_AccelX.get_estimate()
	accel_y = kalman_AccelY.get_estimate()
	accel_z = kalman_AccelZ.get_estimate()

        # Calculate a quaternion representing the raw orientation
        raw_accel = raw_accel_x, raw_accel_y, raw_accel_z
        raw_orientation = get_orientation(raw_accel)	

	# Calculate a quaternion representing the orientation
	accel = accel_x, accel_y, accel_z
	orientation = get_orientation(accel)

	# Read the gyro vals
	raw_gyro_x = read_word_2c(GYRO_XOUT_H) / 131.0
	raw_gyro_y = read_word_2c(GYRO_YOUT_H) / 131.0
	raw_gyro_z = read_word_2c(GYRO_ZOUT_H) / 131.0

        kalman_GyroX.noisy_measurement(raw_gyro_x)
	kalman_GyroY.noisy_measurement(raw_gyro_y)
	kalman_GyroZ.noisy_measurement(raw_gyro_z)

	gyro_x = kalman_GyroX.get_estimate()
	gyro_y = kalman_GyroY.get_estimate()
	gyro_z = kalman_GyroZ.get_estimate()
	    
	# Load up the IMU message
        ts = rospy.Time.now()
	o = imu_msg.orientation
	o.x, o.y, o.z, o.w = orientation

	imu_msg.linear_acceleration.x = accel_x
	imu_msg.linear_acceleration.y = accel_y
	imu_msg.linear_acceleration.z = accel_z

        imu_msg.angular_velocity.x = gyro_x
	imu_msg.angular_velocity.y = gyro_y
	imu_msg.angular_velocity.z = gyro_z

	imu_msg.header.stamp = ts

	imu_pub.publish(imu_msg)

        # Load up the Raw IMU message
        o = raw_imu_msg.orientation
        o.x, o.y, o.z, o.w = raw_orientation

        raw_imu_msg.linear_acceleration.x = raw_accel_x
        raw_imu_msg.linear_acceleration.y = raw_accel_y
        raw_imu_msg.linear_acceleration.z = raw_accel_z

        raw_imu_msg.angular_velocity.x = raw_gyro_x
        raw_imu_msg.angular_velocity.y = raw_gyro_y
        raw_imu_msg.angular_velocity.z = raw_gyro_z

        raw_imu_msg.header.stamp = ts

        raw_imu_pub.publish(raw_imu_msg)
    except:
        print("Unexpected error", sys.exc_info()[0])

temp_pub = None
imu_pub = None

if __name__ == '__main__':
    rospy.init_node('imu_node')

    bus = smbus.SMBus(rospy.get_param('~bus', 1))
    ADDR = rospy.get_param('~device_address', 0x68)
    if type(ADDR) == str:
        ADDR = int(ADDR, 16)

    IMU_FRAME = rospy.get_param('~imu_frame', 'imu_link')

    bus.write_byte_data(ADDR, PWR_MGMT_1, 0)

    temp_pub = rospy.Publisher('temperature', Temperature)
    imu_pub = rospy.Publisher('imu/data', Imu)
    raw_imu_pub = rospy.Publisher('imu/raw_data', Imu)
    imu_timer = rospy.Timer(rospy.Duration(0.02), publish_imu)
    temp_timer = rospy.Timer(rospy.Duration(10), publish_temp)
    rospy.spin()
