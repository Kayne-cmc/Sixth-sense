import numpy as np

from constants import *

def process_frame(depth_buf, amplitude_buf):
    depth_buf = np.nan_to_num(depth_buf)

    amplitude_buf[amplitude_buf<=7] = 0
    amplitude_buf[amplitude_buf>7] = 255

    depth_buf = (1 - (depth_buf/MAX_DISTANCE)) * 255
    depth_buf = np.clip(depth_buf, 0, 255)
    result_frame = depth_buf.astype(np.uint8)  & amplitude_buf.astype(np.uint8)
    return result_frame

# Linear strength adjustment 
# def adjust_motor_vibration(distance, motors, motor_index):
#     if distance < 0.1 or distance > MAX_DISTANCE:  # If distance is too close or invalid, turn off the motor
#         motors[motor_index].ChangeDutyCycle(0)
#     else:
#         # Scale vibration intensity based on distance
#         intensity = (1 - distance / MAX_DISTANCE) * 100  # Convert to duty cycle (0-100%)
#         motors[motor_index].ChangeDutyCycle(intensity)

# Exponential distance strength adjustment