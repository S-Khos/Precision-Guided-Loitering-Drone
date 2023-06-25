from djitellopy import Tello
import cv2
import time
import threading
from state import State
from frontend import FrontEnd
from backend import BackEnd


def main():
    drone = Tello()
    state = State(drone)
    backend = BackEnd(state)
    frontend = FrontEnd(state)
    drone.connect()
    if (drone.get_battery() > 25):
        drone.streamon()
        frame_read = drone.get_frame_read()
        cv2.namedWindow("FEED", cv2.WINDOW_NORMAL)
        # cv2.namedWindow("DESIGNATOR", cv2.WINDOW_NORMAL)
        cv2.moveWindow("FEED", int((1920 // 2) - state.CENTRE_X),
                       int((1080 // 2) - state.CENTRE_Y))
        # cv2.moveWindow("DESIGNATOR", int((1920 // 4) + state.CENTRE_X + 3),
        # int((1080 // 2) - state.CENTRE_Y))
        cv2.setMouseCallback(
            "FEED", backend.cursor_control.event_handler)
        while frame_read:
            state.frame = frame_read.frame
            state.update()
            backend.update()
            frontend.update()
            cv2.imshow("FEED", state.frame)
            # cv2.imshow("DESIGNATOR", state.designator_frame)
            if (cv2.waitKey(1) & 0xff) == 27:
                break

        cv2.destroyAllWindows()
        drone.streamoff()
        drone.end()
    else:
        print("[DRONE] - Low battery")
    drone.end()
    print("[DRONE] - CONNECTION TERMINATED")


if __name__ == "__main__":
    main()
