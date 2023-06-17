import cv2
import time
import math
import threading
from backend import BackEnd


class Tracker(BackEnd):
    def __init__(self):
        super().__init__()

    def init_tracker(self):
        self.designator_frame = self.get_designator_frame()
        self.TR_active = True
        self.TR_reset = False
        self.TR_tracker = cv2.legacy.TrackerCSRT_create()
        self.TR_tracker.init(
            self.designator_frame, (self.CC_cursor_pos[0], self.CC_cursor_pos[1], self.KC_designator_roi_size[0], self.KC_designator_roi_size[1]))

        self.TR_thread = threading.Thread(
            target=self.update, daemon=True)
        self.TR_thread.start()

    def update(self, frame):
        print("[TRACK] - TRACKING ACTIVE")
        try:
            while self.TR_active:
                tracker_ret, self.TR_bbox = self.TR_tracker.update(self.frame)
                if not tracker_ret or self.TR_reset:
                    self.TR_thread_lock.acquire()
                    self.TR_active = False
                    self.TR_reset = True
                    self.TR_thread_lock.release()
        except:
            print("[TRACK] - Invalid Coordinates")
            self.TR_thread_lock.acquire()
            self.TR_active = False
            self.TR_reset = True
            self.TR_thread_lock.release()

        drone.send_rc_control(0, 0, 0, 0)
        print("[TRACK] - TRACKING TERMINATED")

    def get_bbox(self):
        return (self.TR_bbox[0], self.TR_bbox[1], self.TR_bbox[2], self.TR_bbox[3])
