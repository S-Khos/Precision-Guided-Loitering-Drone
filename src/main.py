from tello import Tello
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
        cv2.moveWindow("FEED", int((1920 // 2) - state.FRAME_WIDTH // 2),
                       int((1080 // 2) - state.FRAME_HEIGHT // 2))
        cv2.setMouseCallback(
            "FEED", backend.cursor_control.event_handler)
        while frame_read:
            state.frame = frame_read.frame
            state.frame = cv2.cvtColor(state.frame, cv2.COLOR_BGR2RGB)
            state.designator_frame = state.frame.copy()
            state.update()
            backend.update()
            frontend.update()
            cv2.imshow("FEED", state.frame)
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
