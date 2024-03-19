import RPi.GPIO as GPIO
import time
import sys

from helpers import *
from constants import *

motor_pins = [26, 19, 13, 6]

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(motor_pins, GPIO.OUT)

motors = [GPIO.PWM(motor_pins[0], 100), GPIO.PWM(motor_pins[1], 100), GPIO.PWM(motor_pins[2], 100), GPIO.PWM(motor_pins[3], 100)]

for motor in motors:
    motor.start(0)

for i in range(TOTAL_MOTORS):
    print("Motor " + i + " is vibrating")
    adjust_motor_vibration(0.2, motors, i)
    time.sleep(3)
    adjust_motor_vibration(MAX_DISTANCE, motors, i)
    motors[i].stop()

GPIO.cleanup()