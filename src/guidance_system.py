from djitellopy import Tello
import cv2
import math
import time


class GuidanceSystem(object):

    YAW_PID = [0.32, 0.05, 0.11]  # 0.32, 0, 0.06
    Y_PID = [1.3, 0.18, 0.1]  # 0.1, 0.3, 0.3,
    X_PID = [0.2, 0.0, 0.12]

    def __init__(self, drone, manual_control, tracker, frontend, backend):
        self.drone = drone
        self.manual_control = manual_control
        self.tracker = tracker
        self.frontend = frontend
        self.backend = backend
        self.guidance_sys_active = False
        self.guidance_sys_thread = None

        if not self.guidance_sys_active and not self.manual_control.manual:
            self.guidance_sys_thread = threading.Thread(
                target=self.process(), daemon=True)
            self.guidance_sys_thread.start()
            self.guidance_sys_active = True

    def process(self):
        try:
            print("[FLT CTRL] - ACTIVE")

            yaw_pid = PID(GuidanceSystem.YAW_PID[0], GuidanceSystem.YAW_PID[1], GuidanceSystem.YAW_PID[2],
                          self.frontend.CENTRE_X, -100, 100)
            x_pid = PID(GuidanceSystem.X_PID[0], GuidanceSystem.X_PID[1], GuidanceSystem.X_PID[2],
                        self.frontend.CENTRE_X, -80, 80)
            y_pid = PID(GuidanceSystem.Y_PID[0], GuidanceSystem.Y_PID[1], GuidanceSystem.Y_PID[2],
                        self.frontend.CENTRE_Y, -100, 100)

            while self.tracker.tracking and not self.manual_control.manual:
                x, y, w, h = tracker.get_bbox()
                targetX = x + w // 2
                targetY = y + h // 2

                yaw_velocity, yaw_time = yaw_pid.update(targetX)
                x_velocity, x_time = x_pid.update(targetX)
                y_velocity, y_time = y_pid.update(targetY)

                if drone.send_rc_control:
                    drone.send_rc_control(-x_velocity if abs(x_velocity)
                                          > 60 else 0, 90 if altitude > 1 and self.manual_control.dive else 0, y_velocity, -yaw_velocity)

                time.sleep(0.1)

            yaw_pid.reset()
            y_pid.reset()
            x_pid.reset()
            self.guidance_sys_active = False
            drone.send_rc_control(0, 0, 0, 0)
            print("[FLT CTRL] - TERMINATED")

        except Exception as error:
            yaw_pid.reset()
            y_pid.reset()
            x_pid.reset()
            self.guidance_sys_active = False
            self.manual_control.flip_manual()
            drone.send_rc_control(0, 0, 0, 0)
            print("[FLT CTRL] - Error occured\n", error)
