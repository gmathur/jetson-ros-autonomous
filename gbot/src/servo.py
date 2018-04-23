import RPi.GPIO as GPIO
from time import sleep

class ServoControl:
    PIN = 12
    
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)

        GPIO.setup(ServoControl.PIN, GPIO.OUT)
        self.pwm=GPIO.PWM(ServoControl.PIN, 50)
        self.pwm.start(0)
        self.last_angle = 90

    def setAngle(self, angle):
        if angle == self.last_angle:
            # Nothing to do
            return

        duty = angle / 18 + 2
        self.pwm.ChangeDutyCycle(duty)

        if abs(angle - self.last_angle) > 45:
            sleep(0.35)
        else:
            sleep(0.2)
        self.last_angle = angle

if __name__== "__main__":
    servo = ServoControl()
    servo.setAngle(90)
    servo.setAngle(45)
    servo.setAngle(135)
    servo.setAngle(90)
