from pynput import keyboard
import threading
import cv2


class KeyControl(object):
    def __init__(self, state):
        self.state = state
        self.key_listener = keyboard.Listener(
            on_press=self.on_key_press, on_release=self.on_key_release, daemon=True)
        self.key_listener.start()

    def on_key_press(self, key):
        try:
            if key.char == 'z':
                if self.state.KC_manual and self.state.TR_active:
                    self.state.KC_manual = False
                else:
                    self.state.KC_manual = True
            elif key.char == 'x':
                self.state.GS_dive = not self.state.GS_dive
            elif key.char == 'v':
                self.state.RBG = not self.state.RBG

            if self.state.KC_manual:
                if key.char == 'i':
                    self.state.drone.send_rc_control(0, 0, 0, 0)
                    self.state.drone.takeoff()
                elif key.char == 'k':
                    self.state.drone.send_rc_control(0, 0, 0, 0)
                    self.state.drone.land()
                elif key.char == 'w':
                    self.state.drone.move_forward(self.state.KC_default_dist)
                elif key.char == 's':
                    self.state.drone.move_back(self.state.KC_default_dist)
                elif key.char == 'a':
                    self.state.drone.move_left(self.state.KC_default_dist)
                elif key.char == 'd':
                    self.state.drone.move_right(self.state.KC_default_dist)
                elif key.char == 'q':
                    self.state.drone.rotate_counter_clockwise(45)
                elif key.char == 'e':
                    self.state.drone.rotate_clockwise(45)
                elif key.char == 'r':
                    self.state.drone.move_up(100)
                elif key.char == 'f':
                    self.state.drone.move_down(50)
                elif key.char == ']':
                    self.state.KC_designator_roi_size[0] += self.state.KC_designator_delta
                    self.state.KC_designator_roi_size[1] += self.state.KC_designator_delta
                elif key.char == '[':
                    self.state.KC_designator_roi_size[0] -= self.state.KC_designator_delta
                    self.state.KC_designator_roi_size[1] -= self.state.KC_designator_delta

        except:
            print("[Key Control] - Invalid key.")

    def on_key_release(self, key):
        return True
