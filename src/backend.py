import cv2
import numpy as np
import time
import threading
import math
from djitellopy import Tello


class BackEnd(object):

    manual_ctrl = True
    dive_lock = False

    tracker_thread_lock = threading.Lock()
    tracker_thread = None
    tracker_active = False
    tracker_reset = True
    tracking = False
    tracker_roi = None
    tracker_return = False
    roi_selector_size = [100, 100]
    roi_selector_size_delta = 20
    cursor_pos = [0, 0]

    guidance_sys_active = False
    flt_ctrl_lock = threading.Lock()
    flight_ctrl_thread = None
    dive = False

    YAW_PID = [0.32, 0.05, 0.11]  # 0.32, 0, 0.06
    Y_PID = [1.3, 0.18, 0.1]  # 0.1, 0.3, 0.3,
    X_PID = [0.2, 0.0, 0.12]

    def __init__(self, drone, centre_width, centre_height):
        self.drone = drone
        self.CENTRE_X = centre_width
        self.CENTRE_Y = centre_height
        self.fps_init_time = time.time()
        cursor_pos[0], cursor_pos[1] = self.CENTRE_X, self.CENTRE_Y

    def process(self, frame):

        if tracking and not tracker_active:
            tracker_thread = threading.Thread(
                target=tracker_control, daemon=True)
            tracker_thread.start()

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
        return manual_ctrl

    def guidance_system_active(self):
        return guidance_sys_active
