import cv2
import numpy as np
import time
import math
import threading


class Tracker(object):
    def __init__(self, cursor_control, key_control):
        self.frame = None
        self.cursor_control = cursor_control
        self.key_control = key_control
        self.tracker = None
        self.tracker_thread = None
        self.tracking = False

    def init_tracker(self, frame):
        self.frame = frame
        self.tracker = cv2.legacy.TrackerCSRT_create()
        self.tracker.init(
            self.frame, (self.cursor_control.cursor_pos[0], self.cursor_control.cursor_pos[1], self.key_control.designator_roi_size[0], self.key_control.designator_roi_size[1]))

        self.tracker_thread = threading.Thread(
            target=self.update, daemon=True)
        self.tracker_thread.start()

    def update(self, frame):

        pass

    def terminate(self):
        pass
