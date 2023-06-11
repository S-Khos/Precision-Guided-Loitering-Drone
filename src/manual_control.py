from djitellopy import Tello
from pynput import keyboard
import threading


class KeyControl(object):
    def __init__(self, drone):
        self.drone = drone
        self.dive = False
        self.manual = True
        self.default_dist = 30
        self.designator_delta = 30
        self.designator_roi_size = [100, 100]

        self.key_listener = keyboard.Listener(
            on_press=self.on_press, on_release=self.on_release, daemon=True)
        self.key_listener.start()

    def on_release(self, key):
        return True

    def terminate(self):
        self.key_listener.join()

    def get_manual(self):
        return self.manual

    def flip_manual(self):
        self.manual = not self.manual

    def on_press(self, key):
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
    def __init__(key_control, backend):
        self.key_control = key_control
        self.backend = backend
        self.cursor_pos = [0, 0]

    def event_handler(event, x, y, flags, param):
        cursor_pos[0] = x - self.key_control.designator_roi_size[0] // 2
        cursor_pos[1] = y - self.key_control.designator_roi_size[1] // 2
        if event == cv2.EVENT_LBUTTONDOWN:
            if (not self.backend.tracking and self.backend.reset_track):
                self.backend.reset_track = False
                # tracker = cv2.legacy.TrackerCSRT_create()
                # tracker.init(
                #    empty_frame, (self.cursor_pos[0], cursor_pos[1], designator_roi_size[0], designator_roi_size[1]))
                self.backend.init_tracker = True
                self.backend.tracking = True
            else:
                self.reset_track = True
