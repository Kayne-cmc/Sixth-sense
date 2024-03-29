import RPi.GPIO as GPIO
import time
import sys

from helpers import *
from constants import *
import Classes.Motor as Motor

motor_pins = [26, 19, 13, 6]
motors = []

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(motor_pins, GPIO.OUT)

for i in range(TOTAL_MOTORS):
    motors.append(Motor(GPIO.PWM(motor_pins[i], 100)))

for motor in motors:
    motor.pin.start(0)

for i in range(TOTAL_MOTORS):
    print("Motor " + i + " is vibrating")
    motors[i].update_strength(0.2)
    time.sleep(3)
    motors[i].update_strength(MAX_DISTANCE)
    motors[i].pin.stop()

GPIO.cleanup()