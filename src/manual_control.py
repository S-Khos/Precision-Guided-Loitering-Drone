from djitellopy import Tello
from pynput import keyboard
import threading
import cv2


class KeyControl(object):
    def __init__(self, drone):
        self.drone = drone
        self.dive = False
        self.manual = True
        self.default_dist = 30
        self.designator_delta = 30
        self.designator_roi_size = [100, 100]

        self.key_listener = keyboard.Listener(
            on_press=self.on_key_press, on_release=self.on_key_release, daemon=True)
        self.key_listener.start()

    def on_key_release(self, key):
        return True

    def terminate(self):
        self.key_listener.join()

    def get_manual(self):
        return self.manual

    def flip_manual(self):
        self.manual = not self.manual

    def on_key_press(self, key):
        try:
            if key.char == 'z':
                self.manual = not self.manual
            elif key.char == 'x':
                self.dive = not self.dive
            if self.manual:
                if key.char == 'i':
                    self.drone.send_rc_control(0, 0, 0, 0)
                    self.drone.takeoff()
                elif key.char == 'k':
                    self.drone.send_rc_control(0, 0, 0, 0)
                    self.drone.land()
                elif key.char == 'w':
                    self.drone.move_forward(self.default_dist)
                elif key.char == 's':
                    self.drone.move_back(self.default_dist)
                elif key.char == 'a':
                    self.drone.move_left(self.default_dist)
                elif key.char == 'd':
                    self.drone.move_right(self.default_dist)
                elif key.char == 'q':
                    self.drone.rotate_counter_clockwise(45)
                elif key.char == 'e':
                    self.drone.rotate_clockwise(45)
                elif key.char == 'r':
                    self.drone.move_up(self.default_dist)
                elif key.char == 'f':
                    self.drone.move_down(self.default_dist)
                elif key.char == ']':
                    self.designator_roi_size[0] += self.designator_delta
                    self.designator_roi_size[1] += self.designator_delta
                elif key.char == '[':
                    self.designator_roi_size[0] -= self.designator_delta
                    self.designator_roi_size[1] -= self.designator_delta

        except:
            print("[Manual Control] - Invalid key.")


class CursorControl(object):
    def __init__(self, key_control, tracker):
        self.key_control = key_control
        self.tracker = tracker
        self.cursor_pos = [self.CENTRE_X, self.CENTRE_Y]

    def event_handler(self, event, x, y, flags, param):
        self.cursor_pos[0] = int(
            x - self.key_control.designator_roi_size[0] / 2)
        self.cursor_pos[1] = int(
            y - self.key_control.designator_roi_size[1] / 2)
        if event == cv2.EVENT_LBUTTONDOWN:
            if not self.tracker.active and self.tracker.reset:
                self.tracker.init_tracker()
            else:
                self.tracker.reset = True
                self.tracker.active = False
