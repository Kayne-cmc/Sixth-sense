from constants import *

class Motor:
    def __init__(self, pin, motor_intensity):
        self.pin = pin
        self.motor_intensity = motor_intensity

    def adjust_vibration(self, distance):
        intensity = 0
        
        if distance <= ALERT_DISTANCE:
            intensity = MAX_STRENGTH if self.motor_intensity == 0 else 0
            self.pin.ChangeDutyCycle()
            self.pin.ChangeDutyCycle(100)
        elif distance > MAX_DISTANCE:
            self.pin.ChangeDutyCycle(0)
        else:
            intensity = (1 - (distance / MAX_DISTANCE)**2) * 100  # Convert to duty cycle (0-100%)
            self.pin.ChangeDutyCycle(intensity)
        
        self.pin.ChangeDutyCycle(intensity)
        self.motor_intensity = intensity