import cv2
import time
import math
import threading
from backend import BackEnd


class Tracker(object):
    def __init__(self, backend):
        self.backend = backend

    def init_tracker(self):
        self.backend.TR_reset = False
        self.backend.TR_tracker = cv2.legacy.TrackerCSRT_create()
        self.backend.TR_tracker.init(
            self.backend.get_designator_frame(), (self.backend.CC_cursor_pos[0], self.backend.CC_cursor_pos[1], self.backend.KC_designator_roi_size[0], self.backend.KC_designator_roi_size[1]))

        self.backend.TR_thread = threading.Thread(
            target=self.update, daemon=True)
        self.backend.TR_thread.start()

    def update(self):
        print("[TRACK] - TRACKING ACTIVE")
        try:
            while self.backend.TR_active:
                tracker_ret, self.backend.TR_bbox = self.backend.TR_tracker.update(
                    self.backend.get_designator_frame())
                if not tracker_ret or self.backend.TR_reset:
                    self.backend.TR_thread_lock.acquire()
                    self.backend.TR_active = False
                    self.backend.TR_reset = True
                    self.backend.TR_thread_lock.release()
        except:
            print("[TRACK] - Invalid Coordinates")
            self.backend.TR_thread_lock.acquire()
            self.backend.TR_active = False
            self.backend.TR_reset = True
            self.backend.TR_thread_lock.release()

        drone.send_rc_control(0, 0, 0, 0)
        print("[TRACK] - TRACKING TERMINATED")

    def get_bbox(self):
        return (self.backend.TR_bbox[0], self.backend.TR_bbox[1], self.backend.TR_bbox[2], self.backend.TR_bbox[3])
