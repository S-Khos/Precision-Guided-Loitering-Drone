from djitellopy import Tello
import cv2
import math
import time
from pid import PID


class GuidanceSystem(object):
    def __init__(self, state):
        self.state = state
        self.YAW_PID = [0.32, 0.05, 0.11]  # 0.32, 0, 0.06
        self.Y_PID = [1.3, 0.18, 0.1]  # 0.1, 0.3, 0.3,
        self.X_PID = [0.2, 0.0, 0.12]

    def init_guidance_system(self):
        if not self.state.KC_manual:
            self.state.GS_thread = threading.Thread(
                target=self.process, daemon=True)
            self.state.GS_thread.start()
            self.state.GS_active = True

    def process(self):
        try:
            print("[FLT CTRL] - ACTIVE")

            yaw_pid = PID(self.YAW_PID[0], self.YAW_PID[1], self.YAW_PID[2],
                          self.state.CENTRE_X, -100, 100)
            x_pid = PID(self.X_PID[0], self.X_PID[1], self.X_PID[2],
                        self.state.CENTRE_X, -80, 80)
            y_pid = PID(self.Y_PID[0], self.Y_PID[1], self.Y_PID[2],
                        self.state.CENTRE_Y, -100, 100)

            while self.state.TR_active and not self.state.KC_manual:
                x, y, w, h = self.state.TR_bbox
                targetX = int(x + w / 2)
                targetY = int(y + h / 2)

                yaw_velocity, yaw_time = yaw_pid.update(targetX)
                x_velocity, x_time = x_pid.update(targetX)
                y_velocity, y_time = y_pid.update(targetY)

                if drone.send_rc_control:
                    drone.send_rc_control(-x_velocity if abs(x_velocity)
                                          > 60 else 0, 90 if altitude > 1 and self.state.GS_dive else 0, y_velocity, -yaw_velocity)

                time.sleep(0.1)

            yaw_pid.reset()
            y_pid.reset()
            x_pid.reset()
            self.state.GS_active = False
            drone.send_rc_control(0, 0, 0, 0)
            print("[FLT CTRL] - TERMINATED")

        except Exception as error:
            yaw_pid.reset()
            y_pid.reset()
            x_pid.reset()
            self.state.GS_active = False
            self.state.KC_manual = not self.state.KC_manual
            drone.send_rc_control(0, 0, 0, 0)
            print("[FLT CTRL] - ", error)
