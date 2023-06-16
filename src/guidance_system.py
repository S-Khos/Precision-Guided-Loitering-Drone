from djitellopy import Tello
import cv2
import math
import time


class GuidanceSystem(object):

    YAW_PID = [0.32, 0.05, 0.11]  # 0.32, 0, 0.06
    Y_PID = [1.3, 0.18, 0.1]  # 0.1, 0.3, 0.3,
    X_PID = [0.2, 0.0, 0.12]

    def __init__(self, drone, manual_control, tracker):
        self.drone = drone
        self.manual_control = manual_control
        self.tracker = tracker

        if not guidance_system.flt_ctrl_active and not manual_control.manual:
            flight_ctrl_thread = threading.Thread(
                target=self.guidance_system.process(), daemon=True)
            flight_ctrl_thread.start()
            flt_ctrl_active = True

    def process(self):
        try:
            print("[FLT CTRL] - ACTIVE")

            yaw_pid = PID(YAW_PID[0], YAW_PID[1], YAW_PID[2],
                          frontend.CENTRE_X, -100, 100)
            x_pid = PID(X_PID[0], X_PID[1], X_PID[2],
                        frontend.CENTRE_X, -80, 80)
            y_pid = PID(Y_PID[0], Y_PID[1], Y_PID[2],
                        frontend.CENTRE_Y, -100, 100)

            while cursor_control.tracking and tracker_ret and not manual_control.manual:
                x, y, w, h = [int(value) for value in roi]
                targetX = x + w // 2
                targetY = y + h // 2

                yaw_velocity, yaw_time = yaw_pid.update(targetX)
                x_velocity, x_time = x_pid.update(targetX)
                y_velocity, y_time = y_pid.update(targetY)

                yaw_pid_array.append(yaw_velocity)
                yaw_pid_time.append(yaw_time)

                if drone.send_rc_control:
                    drone.send_rc_control(-x_velocity if abs(x_velocity)
                                          > 60 else 0, 90 if altitude > 1 and manual_control.dive else 0, y_velocity, -yaw_velocity)

                time.sleep(0.1)

            yaw_pid.reset()
            y_pid.reset()
            x_pid.reset()
            flt_ctrl_lock.acquire()
            flt_ctrl_active = False
            flt_ctrl_lock.release()
            drone.send_rc_control(0, 0, 0, 0)
            print("[FLT CTRL] - TERMINATED")

        except Exception as error:
            yaw_pid.reset()
            y_pid.reset()
            x_pid.reset()
            flt_ctrl_lock.acquire()
            flt_ctrl_active = False
            manual_control.flip_manual()
            flt_ctrl_lock.release()
            drone.send_rc_control(0, 0, 0, 0)
            print("[FLT CTRL] - Error occured\n", error)
