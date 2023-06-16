from djitellopy import Tello
import cv2
import math
import time
import threading
from pynput import keyboard
from pid import PID
import matplotlib.pyplot as plt
from manual_control import KeyControl, CursorControl
from frontend import FrontEnd
from tracker import Tracker
from backend import Backend

drone = Tello()
manual_control = KeyControl(drone)
cursor_control = CursorControl(manual_control, frontend)
tracker = Tracker(cursor_control, manual_control, frontend)
frontend = FrontEnd(backend, manual_control, cursor_control, tracker)
backend = BackEnd(drone, frontend, guidance_system, tracker,
                  manual_control, cursor_control)

try:
    drone.connect()
    if (drone.get_battery() > 25):
        drone.streamon()
        frame_read = drone.get_frame_read()
        cv2.namedWindow("FEED", cv2.WINDOW_NORMAL)
        cv2.namedWindow("DESIGNATOR", cv2.WINDOW_NORMAL)
        cv2.moveWindow("FEED", int((1920 // 4) - frontend.CENTRE_X),
                       int((1080 // 2) - frontend.CENTRE_Y))
        cv2.moveWindow("DESIGNATOR", int((1920 // 4) + frontend.CENTRE_X + 10),
                       int((1080 // 2) - frontend.CENTRE_Y))
        cv2.setMouseCallback("DESIGNATOR", cursor_control.event_handler)

        while frame_read:

            frame = frame_read.frame
            backend.process(frame, manual_control)
            frame, designator_frame = frontend.update(frame)

            cv2.imshow("FEED", frame)
            cv2.imshow("DESIGNATOR", designator_frame)
            if (cv2.waitKey(1) & 0xff) == 27:
                break

        cv2.destroyAllWindows()
        drone.streamoff()
        drone.end()

    else:
        print("[DRONE] - Low battery")
        drone.end()

except Exception as error:
    print(error)
    drone.end()

print("[DRONE] - CONNECTION TERMINATED")
