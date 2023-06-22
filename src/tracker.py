import cv2
import time
import math
import threading


class Tracker(object):
    def __init__(self, state):
        self.state = state

    def init_tracker(self):
        self.state.TR_reset = False
        self.state.TR_tracker = cv2.legacy.TrackerCSRT_create()
        self.state.TR_tracker.init(
            self.state.designator_frame, (self.state.CC_cursor_pos[0], self.state.CC_cursor_pos[1], self.state.KC_designator_roi_size[0], self.state.KC_designator_roi_size[1]))

        self.state.TR_thread = threading.Thread(
            target=self.update, daemon=True)
        self.state.TR_thread.start()

    def update(self):
        print("[TRACK] - TRACKING ACTIVE")
        try:
            while self.state.TR_active:
                tracker_ret, self.state.TR_bbox = self.state.TR_tracker.update(
                    self.state.get_designator_frame())
                if not tracker_ret or self.state.TR_reset:
                    self.state.TR_thread_lock.acquire()
                    self.state.TR_active = False
                    self.state.TR_reset = True
                    self.state.TR_thread_lock.release()
        except:
            print("[TRACK] - Invalid Coordinates")
            self.state.TR_thread_lock.acquire()
            self.state.TR_active = False
            self.state.TR_reset = True
            self.state.TR_thread_lock.release()

        drone.send_rc_control(0, 0, 0, 0)
        print("[TRACK] - TRACKING TERMINATED")
