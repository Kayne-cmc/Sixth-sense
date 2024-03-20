from constants import *
import time

class Motor:
    def __init__(self, pin, motor_intensity):
        self.pin = pin
        self.intensity = motor_intensity
        self.pulsing_frequency = None
        self.pattern = None
        self.start_time = None
        self.section_depth = 0

    def update_pattern(self):
        # Only need to set a pulsing vibration if withing ALERT_DISTANCE
        if self.section_depth <= ALERT_DISTANCE:
            current_time = time.time()
            if self.pattern == Patterns.PULSE:
                # Need to switch on/off pulsating motor
                if (current_time - self.start_time > PULSE_DURATION):
                    intensity = 0 if self.intensity > 0 else MAX_STRENGTH
                    self.update_pulse_frequency()
                    self.start_time = current_time
                    self.intensity = intensity
            else:
                self.pattern = Patterns.PULSE
                self.update_pulse_frequency()
                self.start_time = current_time
        else:
            self.pattern = None
            self.pulsing_frequency = None
            self.intensity = 0

    def update_strength(self):
        if self.pattern is not None:
            return
        
        intensity = 0
        if self.section_depth > MAX_DISTANCE:
            intensity = 0
        else:
            intensity = (1 - (self.section_depth / MAX_DISTANCE)) ** 2 * 100 # 0 - 100 intensity from a distance of 4m - 0m
        self.intensity = intensity

    def update(self):
        self.update_pattern()
        self.update_strength()
    
    def update_pulse_frequency(self):
        self.pulse_frequency = self.section_depth/2

    def apply_updates(self):
        self.pin.ChangeDutyCycle(self.intensity)