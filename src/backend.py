import cv2
import numpy as np
import time
import threading
import math
from djitellopy import Tello
from tracker import Tracker
from manual_control import ManualControl, CursorControl
from frontend import FrontEnd
from tracker import Tracker


class BackEnd(object):

    def __init__(self, drone, frontend, guidance_system, tracker, manual_control, cursor_control):
        self.drone = drone
        self.guidance_system = guidance_system
        self.tracker = tracker
        self.frontend = frontend
        self.manual_control = manual_control
        self.cursor_control = cursor_control
        self.fps_init_time = time.time()

    def process(self, frame):
        pass

    def get_fps(self):
        elapsed_time = time.time() - self.fps_init_time
        fps = int(1 / elapsed_time)
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
