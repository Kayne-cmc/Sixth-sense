from constants import *
import time

class Motor:
    def __init__(self, pin, motor_intensity):
        self.pin = pin
        self.intensity = motor_intensity
        self.pattern = None
        self.start_time = None

    def update_pattern(self, distance):
        # Only need to set a pulsing vibration if withing ALERT_DISTANCE
        if distance <= ALERT_DISTANCE:
            current_time = time.time()
            if self.pattern == Patterns.PULSE:
                # Need to switch on/off pulsating motor
                if (current_time - self.start_time > PULSE_DURATION):
                    intensity = 0 if self.intensity > 0 else MAX_STRENGTH
                    self.start_time = current_time
                    self.intensity = intensity
            else:
                self.pattern = Patterns.PULSE
                self.start_time = current_time
        else:
            self.pattern = None
            self.intensity = 0

    def update_strength(self, distance):
        if self.pattern is not None:
            return
        
        intensity = 0
        if distance > MAX_DISTANCE:
            intensity = 0
        else:
            intensity = (1 - (distance / MAX_DISTANCE)**2) * 100 # 0 - 100 intensity from a distance of 4m - 0m
        self.intensity = intensity

    def update(self, distance):
        self.update_pattern(distance)
        self.update_strength(distance)
        self.pin.ChangeDutyCycle(self.intensity)