import cv2
import time
import threading


class Tracker(object):
    def __init__(self, state):
        self.state = state

    def init_tracker(self):
        self.state.TR_reset = False
        self.state.TR_tracker = cv2.legacy.TrackerCSRT_create()
        self.state.TR_tracker.init(
            self.state.frame, (self.state.CC_cursor_pos[0], self.state.CC_cursor_pos[1], self.state.KC_designator_roi_size[0], self.state.KC_designator_roi_size[1]))

        self.state.TR_thread = threading.Thread(
            target=self.update, daemon=True)
        self.state.TR_thread.start()

    def update(self):
        print("[TRACK] - TRACKING ACTIVE")
        try:
            while self.state.TR_active:
                self.state.TR_thread_lock.acquire()
                self.state.TR_return, self.state.TR_bbox = self.state.TR_tracker.update(
                    self.state.frame)
                self.state.TR_thread_lock.release()
                if not self.state.TR_return:
                    self.state.TR_thread_lock.acquire()
                    self.state.TR_active = False
                    self.state.TR_reset = True
                    self.state.TR_thread_lock.release()
        except:
            print("[TRACK] - Invalid Input")
            self.state.TR_thread_lock.acquire()
            self.state.TR_active = False
            self.state.TR_reset = True
            self.state.TR_thread_lock.release()

        self.state.TR_thread_lock.acquire()
        self.state.TR_active = False
        self.state.TR_reset = True
        self.state.TR_thread_lock.release()
        self.state.drone.send_rc_control(0, 0, 0, 0)
        print("[TRACK] - TRACKING TERMINATED")
