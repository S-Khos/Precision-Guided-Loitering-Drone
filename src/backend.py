import cv2
import numpy as np
import time
import threading
import math
from djitellopy import Tello
from tracker import Tracker
from manual_control import ManualControl


class BackEnd(object):

    tracker_thread_lock = threading.Lock()
    tracker_thread = None
    tracker_active = False
    tracker_reset = True
    tracking = False
    tracker_roi = None
    tracker_return = False

    guidance_sys_active = False
    flt_ctrl_lock = threading.Lock()
    flight_ctrl_thread = None

    YAW_PID = [0.32, 0.05, 0.11]  # 0.32, 0, 0.06
    Y_PID = [1.3, 0.18, 0.1]  # 0.1, 0.3, 0.3,
    X_PID = [0.2, 0.0, 0.12]

    def __init__(self, drone, centre_width, centre_height):
        self.drone = drone
        self.CENTRE_X = centre_width
        self.CENTRE_Y = centre_height
        self.fps_init_time = time.time()
        self.manual_control = None

    def process(self, frame, manual_control):
        self.manual_control = manual_control

        if tracking and not tracker_active:
            tracker_thread = threading.Thread(
                target=tracker_control, daemon=True)
            tracker_thread.start()

        if not tracking and not tracker_active and tracker_thread:
            print("[TRACK] - TRACKING RESET")
            tracker_thread = None

        if not flt_ctrl_active and not manual_ctrl:
            flight_ctrl_thread = threading.Thread(
                target=guidance_system, daemon=True)
            flight_ctrl_thread.start()
            flt_ctrl_active = True

    def is_tracking(self):
        return tracking

    def is_tracker_active(self):
        return tracker_active

    def is_tracker_reset(self):
        return tracker_reset

    def get_tracker_return(self):
        return tracker_return

    def get_fps(self):
        elapsed_time = time.time() - self.fps_init_time
        fps = 1 / elapsed_time
        self.fps_init_time = time.time()
        return fps

    def get_altitude(self):
        return self.drone.get_distance_tof() / 30.48

    def get_speed_mag(self):
        return abs(math.sqrt(self.drone.get_speed_x()**2 + self.drone.get_speed_y()**2 + self.drone.get_speed_z()**2))

    def get_battery(self):
        return int(self.drone.get_battery())

    def get_temperature(self):
        return int(self.drone.get_temperature())

    def get_flight_time(self):
        return int(self.drone.get_flight_time())

    def get_barometer(self):
        return int(self.drone.get_barometer())

    def get_acceleration_x(self):
        return int(self.drone.get_acceleration_x())

    def get_acceleration_y(self):
        return int(self.drone.get_acceleration_y())

    def get_acceleration_z(self):
        return int(self.drone.get_acceleration_z())

    def get_speed_x(self):
        return int(self.drone.get_speed_x())

    def get_speed_y(self):
        return int(self.drone.get_speed_y())

    def get_speed_z(self):
        return int(self.drone.get_speed_z())

    def get_yaw(self):
        return int(self.drone.get_yaw())

    def get_pitch(self):
        return int(self.drone.get_pitch())

    def get_roll(self):
        return int(self.drone.get_roll())

    def in_manual_ctrl(self):
        return self.manual_control.manual

    def guidance_system_active(self):
        return guidance_sys_active
