from djitellopy import Tello
from pynput import keyboard
import threading


class ManualControl(object):
    def __init__(self, drone):
        self.drone = drone
        self.dive = False
        self.manual = True
        self.default_dist = 30
        self.designator_delta = 30
        self.designator_roi_size = [100, 100]

        self.key_listener = keyboard.Listener(
            on_press=self.on_press, on_release=self.on_release)
        key_listener.start()

    def on_release(self, key):
        if key == 'Key.esc':
            return False

    def on_press(self, key):
        try:
            if key == 'z':
                self.manual = not self.manual
            elif key == 'x':
                self.dive = not self.dive
            if self.manual:
                if key == 'i':
                    self.drone.send_rc_control(0, 0, 0, 0)
                    self.drone.takeoff()
                elif key == 'k':
                    self.drone.send_rc_control(0, 0, 0, 0)
                    self.drone.land()
                elif key == 'w':
                    self.drone.move_forward(self.default_dist)
                elif key == 's':
                    self.drone.move_back(self.default_dist)
                elif key == 'a':
                    self.drone.move_left(self.default_dist)
                elif key == 'd':
                    self.drone.move_right(self.default_dist)
                elif key == 'q':
                    self.drone.rotate_counter_clockwise(45)
                elif key == 'e':
                    self.drone.rotate_clockwise(45)
                elif key == "Key.up":
                    self.drone.move_up(self.default_dist)
                elif key == "Key.down":
                    self.drone.move_down(self.default_dist)
                elif key == "Key.left":
                    self.designator_roi_size[0] += self.designator_delta
                    self.designator_roi_size[1] += self.designator_delta
                elif key == "Key.right":
                    self.designator_roi_size[0] -= self.designator_delta
                    self.designator_roi_size[1] -= self.designator_delta

        except:
            print("[ManualControl] - Invalid key.")
