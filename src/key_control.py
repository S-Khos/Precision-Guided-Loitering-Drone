from djitellopy import Tello
from pynput import keyboard
import threading
import cv2


class KeyControl(object):
    def __init__(self, state):
        self.state = state

        self.key_listener = keyboard.Listener(
            on_press=self.on_key_press, on_release=self.on_key_release, daemon=True)
        self.key_listener.start()

    def on_key_release(self, key):
        return True

    def on_key_press(self, key):
        try:
            if key.char == 'z':
                self.state.KC_manual = not self.state.KC_manual
            elif key.char == 'x':
                self.dive = not self.dive
            if self.state.KC_manual:
                if key.char == 'i':
                    self.drone.send_rc_control(0, 0, 0, 0)
                    self.drone.takeoff()
                elif key.char == 'k':
                    self.drone.send_rc_control(0, 0, 0, 0)
                    self.drone.land()
                elif key.char == 'w':
                    self.drone.move_forward(self.state.KC_default_dist)
                elif key.char == 's':
                    self.drone.move_back(self.state.KC_default_dist)
                elif key.char == 'a':
                    self.drone.move_left(self.state.KC_default_dist)
                elif key.char == 'd':
                    self.drone.move_right(self.state.KC_default_dist)
                elif key.char == 'q':
                    self.drone.rotate_counter_clockwise(45)
                elif key.char == 'e':
                    self.drone.rotate_clockwise(45)
                elif key.char == 'r':
                    self.drone.move_up(self.state.KC_default_dist)
                elif key.char == 'f':
                    self.drone.move_down(self.state.KC_default_dist)
                elif key.char == ']':
                    self.state.KC_designator_roi_size[0] += self.state.KC_designator_delta
                    self.state.KC_designator_roi_size[1] += self.state.KC_designator_delta
                elif key.char == '[':
                    self.state.KC_designator_roi_size[0] -= self.state.KC_designator_delta
                    self.state.KC_designator_roi_size[1] -= self.state.KC_designator_delta

        except:
            print("[Key Control] - Invalid key.")
