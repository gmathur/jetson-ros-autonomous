#!/usr/bin/env python
import time
import serial

class DimensionDriver:
    MOTOR_1_MULTIPLIER = 0.85
    MOTOR_2_MULTIPLIER = 1

    def __init__(self, address, port):
        self.address = 128
        self.ser = serial.Serial(
            port=port, #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
            baudrate = 38400,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

        self.send_command(15, 4)
      
    def open(self):
        if not self.ser.is_open:
            self.ser.open()
        time.sleep(0.2)

        # Set a 3 second timeout
        self.send_command(14, 30)

        # Fast ramping of 1/8 seconds
        self.send_command(16, 2)

    def send_command(self, cmd, speed):
        values = [self.address, cmd, speed, (self.address + cmd + speed) & 127]
        msg = bytes(bytearray(values))
        self.ser.write(msg)
        self.ser.flush()

    def get_speeds(self, speed):
        return int(speed * DimensionDriver.MOTOR_1_MULTIPLIER), int(speed * DimensionDriver.MOTOR_2_MULTIPLIER)

    def drive_forward(self, speed):
        left, right = self.get_speeds(speed)
        self.send_command(0, left)
        self.send_command(4, right)

    def drive_backward(self, speed):
        left, right = self.get_speeds(speed)
        self.send_command(1, left)
        self.send_command(5, right)

    def turn_right(self, speed):
        left, right = self.get_speeds(speed)
        self.send_command(0, left)
        self.send_command(5, right)
    
    def turn_left(self, speed):
        left, right = self.get_speeds(speed)
        self.send_command(1, left)
        self.send_command(4, right)

    def stop(self):
        self.drive_forward(0)

    def close(self):
        self.ser.close()

if __name__== "__main__":
    driver = DimensionDriver(128, '/dev/ttyUSB0')
    driver.open()
    driver.drive_forward(127)
    time.sleep(1)
    driver.drive_backward(127)
    time.sleep(1)
    driver.turn_right(127)
    time.sleep(1)
    driver.turn_left(127)
    time.sleep(1)
    driver.stop()

    driver.close()
