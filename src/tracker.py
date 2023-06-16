import cv2
import time
import math
import threading


class Tracker(object):
    def __init__(self, cursor_control, key_control, frontend):
        self.cursor_control = cursor_control
        self.key_control = key_control
        self.frontend = frontend
        self.active = False
        self.reset = True
        self.tracker = None
        self.thread = None
        self.designator_frame = None
        self.bbox = None
        self.thread_lock = threading.Lock()

    def init_tracker(self):
        self.designator_frame = self.frontend.get_designator_frame()
        self.active = True
        self.reset = False
        self.tracker = cv2.legacy.TrackerCSRT_create()
        self.tracker.init(
            self.designator_frame, (self.cursor_control.cursor_pos[0], self.cursor_control.cursor_pos[1], self.key_control.designator_roi_size[0], self.key_control.designator_roi_size[1]))

        self.thread = threading.Thread(
            target=self.update, daemon=True)
        self.thread.start()

    def update(self, frame):
        print("[TRACK] - TRACKING ACTIVE")
        try:
            while self.active:
                tracker_ret, self.bbox = tracker.update(frame)
                if not tracker_ret or self.reset:
                    self.thread_lock.acquire()
                    self.active = False
                    self.reset = True
                    self.thread_lock.release()
        except:
            print("[TRACK] - Invalid Coordinates")
            self.thread_lock.acquire()
            self.active = False
            self.reset = True
            self.thread_lock.release()

        drone.send_rc_control(0, 0, 0, 0)
        print("[TRACK] - TRACKING TERMINATED")

    def get_bbox(self):
        return (self.bbox[0], self.bbox[1], self.bbox[2], self.bbox[3])
