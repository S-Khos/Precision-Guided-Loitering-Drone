import cv2
import time
import math
import threading


class Tracker(object):
    def __init__(self, cursor_control, key_control, frontend):
        self.cursor_control = cursor_control
        self.key_control = key_control
        self.frontend = frontend
        self.tracking = False
        self.reset_tracker = True
        self.tracker = None
        self.tracker_thread = None
        self.designator_frame = None
        self.bbox = None

    def init_tracker(self, designator_frame):
        self.designator_frame = designator_frame
        self.tracking = True
        self.reset_tracker = False
        self.thread_lock = threading.Lock()
        self.tracker = cv2.legacy.TrackerCSRT_create()
        self.tracker.init(
            self.designator_frame, (self.cursor_control.cursor_pos[0], self.cursor_control.cursor_pos[1], self.key_control.designator_roi_size[0], self.key_control.designator_roi_size[1]))

        self.tracker_thread = threading.Thread(
            target=self.update, daemon=True)
        self.tracker_thread.start()

    def update(self, frame):
        print("[TRACK] - TRACKING ACTIVE")
        try:
            while self.tracking:
                tracker_ret, self.bbox = tracker.update(frame)
                if not tracker_ret or self.reset_tracker:
                    self.thread_lock.acquire()
                    self.tracking = False
                    self.reset_tracker = True
                    self.thread_lock.release()
        except:
            print("[TRACK] - Invalid Coordinates")
            self.thread_lock.acquire()
            self.tracking = False
            self.reset_tracker = True
            self.thread_lock.release()

        drone.send_rc_control(0, 0, 0, 0)
        print("[TRACK] - TRACKING TERMINATED")
