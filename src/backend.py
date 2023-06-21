import cv2
import numpy as np
import time
import threading
import math
from djitellopy import Tello
from cursor_control import CursorControl
from key_control import KeyControl
from tracker import Tracker
from guidance_system import GuidanceSystem


class BackEnd(object):

    def __init__(self, state):
        self.state = state
        self.key_control = KeyControl(self.state)
        self.tracker = Tracker(self.state)
        self.cursor_control = CursorControl(self.state, self.tracker)
        self.guidance_system = GuidanceSystem(self.state)

        self.fps_init_time = time.time()

    def update(self):
        # init tracking
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

    def in_manual_control(self):
        return (self.state.KC_manual and not self.state.GS_active)
