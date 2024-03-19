import sys
import cv2
import numpy as np
import ArducamDepthCamera as ac
import RPi.GPIO as GPIO
import time
import threading

from helpers import *
from constants import *

class Project:
    def __init__(self, motor_pins, button_pin):
        self.running_thread = None
        self.is_running = threading.Event()
        self.setup_components(motor_pins, button_pin)

        # FOR TEST
        # self.distances = [4,3,2,0.5]
        
    def setup_components(self, motor_pins, button_pin):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Set up motors and button
        GPIO.setup(motor_pins, GPIO.OUT)
        GPIO.setup(button_pin, GPIO.IN, pull_up_down = GPIO.PUD_UP)
        self.motors = [GPIO.PWM(motor_pins[0], 100), GPIO.PWM(motor_pins[1], 100), GPIO.PWM(motor_pins[2], 100), GPIO.PWM(motor_pins[3], 100)]
        GPIO.add_event_detect(button_pin, GPIO.FALLING, callback = self.toggle_start, bouncetime = 100)

        # Set up camera -- UNCOMMENT
        # self.cam = ac.ArducamCamera()
        # if self.cam.open(ac.TOFConnect.CSI,0) != 0 :
        #     print("initialization failed")
        # if self.cam.start(ac.TOFOutput.DEPTH) != 0 :
        #     print("Failed to start camera")
        # self.cam.setControl(ac.TOFControl.RANG,MAX_DISTANCE)

    def start_run(self):
        self.is_running.set()
        print("starting run")

        cv2.namedWindow("preview", cv2.WINDOW_NORMAL)

        for motor in self.motors:
            motor.start(0)

        while self.is_running.isSet():
            # frame = self.cam.requestFrame(200)
            # if frame != None:
            #     depth_buf = frame.getDepthData()
            #     amplitude_buf = frame.getAmplitudeData()
            #     self.cam.releaseFrame(frame)

            #     amplitude_buf *= (255 / 1024)
            #     amplitude_buf = np.clip(amplitude_buf, 0, 255)

            #     result_image = process_frame(depth_buf, amplitude_buf)
            #     result_image = cv2.applyColorMap(result_image, cv2.COLORMAP_JET)
            #     result_image = cv2.rotate(result_image, cv2.ROTATE_180)

            #     frame_height, frame_width = depth_buf.shape
            #     section_width = frame_width // TOTAL_MOTORS
            #     for i in range(TOTAL_MOTORS):
            #         start_x = i * section_width
            #         end_x = ((i + 1) * section_width) if (i < 4) else result_image.shape[1]
            #         section = depth_buf[:, start_x:end_x]
            #         section_depth = np.nanmean(np.where(section == 0, np.nan, section))
            #         # cv2.putText(result_image, f"{i}: {section_depth:.1f}m", (start_x + 10, frame_height//2), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 1)
            #         adjust_motor_vibration(section_depth, self.motors, i)

            #         # Calculate the vibration intensity for visual representation
            #         if section_depth < 0.1 or section_depth > MAX_DISTANCE:
            #             color_intensity = 0  # Black for no vibration
            #         else:
            #             intensity = (1 - section_depth / MAX_DISTANCE) * 255  # Scale from 0 (no vibration) to 255 (max vibration)
            #             color_intensity = int(intensity)
                    
            #         # Simulate vibration by pulsating color intensity
            #         rectangle_color = (color_intensity, color_intensity, color_intensity)  # Grayscale intensity

            #         # Draw rectangle with calculated color
            #         cv2.rectangle(result_image, (start_x, 0), (end_x, 20), rectangle_color, -1)  # Fixed height, color changes

            #     cv2.imshow("preview", result_image)
            #     key = cv2.waitKey(1)


            # FOR TEST -- NEED TO REMOVE
            # for j in range(len(self.distances)):
            #     print("Distance is {}".format(self.distances[j]))
            #     for i in range(TOTAL_MOTORS):
            #         adjust_motor_vibration(self.distances[j], self.motors, i)

            # adjust_motor_vibration(self.distances[3], self.motors, 0)
            adjust_motor_vibration(self.distances[1], self.motors, 3)
            time.sleep(100)
            # END OF TEST


        cv2.destroyAllWindows()


    def stop_run(self):
        self.is_running.clear()
        print("stopping run")
        
        for motor in self.motors:
            motor.stop()

    def shutdown(self):
        self.stop_run()
        # if hasattr(self, 'cam'):
        #     self.cam.stop()
        #     self.cam.close()
        GPIO.cleanup()
        sys.exit(0)

    def toggle_start(self, null):
        if (self.is_running.isSet()):
            self.stop_run()
        else:
            self.running_thread = threading.Thread(target = self.start_run)
            self.running_thread.start()

if __name__ == "__main__":
    button_pin = 4
    motor_pins = [26, 19, 13, 6] # 5 is the remaining pin
    project = Project(motor_pins, button_pin)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        project.shutdown()
        sys.exit(0)