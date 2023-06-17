from djitellopy import Tello
from pynput import keyboard
import threading
import cv2


class KeyControl(BackEnd):
    def __init__(self):
        super().__init__()

        self.key_listener = keyboard.Listener(
            on_press=self.on_key_press, on_release=self.on_key_release, daemon=True)
        self.key_listener.start()

    def on_key_release(self, key):
        return True

    def get_manual(self):
        return self.KC_manual

    def flip_manual(self):
        self.KC_manual = not self.KC_manual

    def on_key_press(self, key):
        try:
            if key.char == 'z':
                self.KC_manual = not self.KC_manual
            elif key.char == 'x':
                self.dive = not self.dive
            if self.KC_manual:
                if key.char == 'i':
                    self.drone.send_rc_control(0, 0, 0, 0)
                    self.drone.takeoff()
                elif key.char == 'k':
                    self.drone.send_rc_control(0, 0, 0, 0)
                    self.drone.land()
                elif key.char == 'w':
                    self.drone.move_forward(self.KC_default_dist)
                elif key.char == 's':
                    self.drone.move_back(self.KC_default_dist)
                elif key.char == 'a':
                    self.drone.move_left(self.KC_default_dist)
                elif key.char == 'd':
                    self.drone.move_right(self.KC_default_dist)
                elif key.char == 'q':
                    self.drone.rotate_counter_clockwise(45)
                elif key.char == 'e':
                    self.drone.rotate_clockwise(45)
                elif key.char == 'r':
                    self.drone.move_up(self.KC_default_dist)
                elif key.char == 'f':
                    self.drone.move_down(self.KC_default_dist)
                elif key.char == ']':
                    self.KC_designator_roi_size[0] += self.KC_designator_delta
                    self.KC_designator_roi_size[1] += self.KC_designator_delta
                elif key.char == '[':
                    self.KC_designator_roi_size[0] -= self.KC_designator_delta
                    self.KC_designator_roi_size[1] -= self.KC_designator_delta

        except:
            print("[Key Control] - Invalid key.")
