#!/usr/bin/env python
import time
import math
import RPi.GPIO as GPIO

TRIGGER_PIN = 18
SENSOR_PINS = [19]
SAMPLE_WAIT=0.1

class SensorState:
    WAIT_FOR_START = 0
    STARTED = 1
    STOPPED = 2
    state_map = [
        self.wait_for_start,
        self.started,
        self.stopped
    ]

    def __init__(self):
        self.start = 0
        self.end = 0
        self.state = SensorState.WAIT_FOR_START

    def get_dist(self, speed_of_sound):
        return (self.end - self.start) * ((speed_of_sound * 100) / 2)

    def execute(self):
        return self.state_map[self.state]()

    def wait_for_start(self):
        if GPIO.input(pin) != 0:
            current_pin_state.start = time.time()
            current_pin_state.state += 1
        #else keep waiting while 0

        return 0 # not finished

    def started(self):
        if GPIO.input(pin) != 1:
            current_pin_state.end = time.time()
            current_pin_state.state += 1
        #else keep waiting while 1

        return 0 # not finished

    def stopped(self):
        return 1 # finished

class UltrasonicScanner:
    def __init__(self, temperature=20.0):
        self.speed_of_sound = 331.3 * math.sqrt(1+(temperature / 273.15))
        GPIO.setwarnings(False)
        GPIO.setup(TRIGGER_PIN, GPIO.OUT)
        for pin in SENSOR_PINS:
            GPIO.setup(pin, GPIO.IN)

    def stop(self):
        GPIO.cleanup(TRIGGER_PIN)
        GPIO.cleanup(SENSOR_PINS)

    def scan(self, sample_size=3):
        samples_runs = []
        
        for _ in range(sample_size):
            sensor_states = [SensorState()] * len(SENSOR_PINS)

            # Trigger echo
            GPIO.output(TRIGGER_PIN, False)
            time.sleep(SAMPLE_WAIT)
            GPIO.output(TRIGGER_PIN, True)
            time.sleep(0.00001)
            GPIO.output(TRIGGER_PIN, False)

            completed = 0
            while(completed < len(SENSOR_PINS)):
                completed = 0

                for pin in SENSOR_PINS:
                    completed += sensor_states[pin].execute()

            samples_runs.append(sensor_states)

        # Results
        final_results = [0.0] * len(SENSOR_PINS)

        # Compute states
        for sample in range(sample_size):
            for state in sample:
                final_results[i] += state.get_dist(self.speed_of_sound)

        # Average
        return tuple([ x / sample_size for x in final_results ])

if __name__ == "__main__":
    scanner = UltrasonicScanner()
    scanner.scan()
