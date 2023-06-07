import threading
import time
from pynput import keyboard


class KeyboardEvent():
    def __init__(self):
        self.__key = None
        self.__key_pressed = False
        self.__listener = None
        self.__thread = None


def manual_controller(key):
    global manual_control, drone, default_dist, dive
    try:
        if key.char == 'z':
            if manual_control:
                manual_control = False
                drone.send_rc_control(0, 0, 0, 0)
            else:
                manual_control = True
                drone.send_rc_control(0, 0, 0, 0)

        if key.char == 'x':
            if dive != True:
                dive = True
            else:
                dive = False

        if manual_control:
            if key.char == 'i':
                drone.send_rc_control(0, 0, 0, 0)
                drone.takeoff()
            elif key.char == 'k':
                drone.land()
            elif key.char == 'w':
                drone.move_forward(default_dist)
            elif key.char == 'a':
                drone.move_left(default_dist)
            elif key.char == 'd':
                drone.move_right(default_dist)
            elif key.char == 's':
                drone.move_back(default_dist)
            elif key.char == 'q':
                drone.rotate_counter_clockwise(default_dist)
            elif key.char == 'e':
                drone.rotate_clockwise(default_dist)
            elif key.char == 'up':
                drone.move_up(default_dist)
            elif key.char == 'down':
                drone.move_down(default_dist)
    except:
        print("[MNUL CTRL] - Invalid key")
