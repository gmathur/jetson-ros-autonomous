#!/usr/bin/env python
import time
import math
import RPi.GPIO as GPIO

TRIGGER_PIN = 17
SENSOR_PINS = [22, 27] # left, right
SAMPLE_WAIT=0.1

class SensorState:
    WAIT_FOR_START = 0
    STARTED = 1
    STOPPED = 2

    def __init__(self, pin):
        self.pin = pin
        self.start = 0
        self.end = 0
        self.state = SensorState.WAIT_FOR_START
        self.state_map = [
            self.wait_for_start,
            self.started,
            self.stopped
        ]

    def get_dist(self, speed_of_sound):
        return (self.end - self.start) * ((speed_of_sound * 100) / 2)

    def execute(self):
        return self.state_map[self.state]()

    def wait_for_start(self):
        if GPIO.input(self.pin) != 0:
            self.start = time.time()
            self.state += 1
        #else keep waiting while 0

        return 0 # not finished

    def started(self):
        if GPIO.input(self.pin) != 1:
            self.end = time.time()
            self.state += 1
        #else keep waiting while 1

        return 0 # not finished

    def stopped(self):
        return 1 # finished

    def __repr__(self):
        return("State %s start %s end %s dist %f" % (self.state, self.start, self.end, self.get_dist(330)))

class UltrasonicScanner:
    def __init__(self, temperature=20.0):
        self.speed_of_sound = 331.3 * math.sqrt(1+(temperature / 273.15))
        self.num_sensors = len(SENSOR_PINS)
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIGGER_PIN, GPIO.OUT)
        for pin in SENSOR_PINS:
            GPIO.setup(pin, GPIO.IN)

    def stop(self):
        GPIO.cleanup(TRIGGER_PIN)
        GPIO.cleanup(SENSOR_PINS)

    def scan(self, sample_size=3):
        samples = []
        
        for _ in range(sample_size):
            sensor_states = []
            for pin in SENSOR_PINS:
                sensor_states.append(SensorState(pin))

            # Trigger echo
            GPIO.output(TRIGGER_PIN, False)
            time.sleep(SAMPLE_WAIT)
            GPIO.output(TRIGGER_PIN, True)
            time.sleep(0.00001)
            GPIO.output(TRIGGER_PIN, False)

            completed = 0
            start_time = time.time()
            while(completed < self.num_sensors):
                completed = 0

                for pin in range(self.num_sensors):
                    completed += sensor_states[pin].execute()

                if time.time() - start_time > 1000:
                    return

            samples.append(sensor_states)

        # Results
        final_results = [0.0] * self.num_sensors

        # Compute states
        for i in range(self.num_sensors):
            for sample in samples:
                #print sample[i].get_dist(self.speed_of_sound), final_results[i]
                final_results[i] += sample[i].get_dist(self.speed_of_sound)

        # Average
        return [ x / sample_size for x in final_results ]

if __name__ == "__main__":
    scanner = UltrasonicScanner()
    dist = scanner.scan()
    print dist
