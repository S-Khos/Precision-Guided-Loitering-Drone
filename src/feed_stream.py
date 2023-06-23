from djitellopy import Tello
import cv2
import time
import threading
from state import State
from frontend import FrontEnd
from backend import BackEnd


class FeedStream(object):
    def __init__(self):
        self.drone = Tello()
        self.state = State(self.drone)
        self.backend = BackEnd(self.state)
        self.frontend = FrontEnd(self.state)
        self.thread_lock = threading.Lock()
        self.drone.connect()
        if (self.drone.get_battery() > 25):
            self.drone.streamon()
            self.frame_read = self.drone.get_frame_read()
            self.thread = threading.Thread(target=self.update, daemon=True)
            self.thread.start()
        else:
            print("[DRONE] - Low battery")
            self.drone.end()

    def update(self):
        while True:
            if self.frame_read:
                self.thread_lock.acquire()
                self.state.frame = self.frame_read.frame
                self.thread_lock.release()
           # time.sleep(0.01)

        self.drone.streamoff()
        self.drone.end()

    def show_frame(self):
        cv2.namedWindow("FEED", cv2.WINDOW_NORMAL)
        cv2.namedWindow("DESIGNATOR", cv2.WINDOW_NORMAL)
        cv2.moveWindow("FEED", int((1920 // 4) - self.state.CENTRE_X + 3),
                       int((1080 // 2) - self.state.CENTRE_Y))
        cv2.moveWindow("DESIGNATOR", int((1920 // 4) + self.state.CENTRE_X + 5),
                       int((1080 // 2) - self.state.CENTRE_Y))

        cv2.setMouseCallback(
            "DESIGNATOR", self.backend.cursor_control.event_handler)

        self.state.update()
        self.backend.update()
        self.frontend.update()

        cv2.imshow("FEED", self.state.frame)
        cv2.imshow("DESIGNATOR", self.state.designator_frame)
        if (cv2.waitKey(1) & 0xff) == 27:
            cv2.destroyAllWindows()
            self.drone.streamoff()
            self.drone.end()


if __name__ == "__main__":
    feed_stream = FeedStream()
    while True:
        try:
            feed_stream.show_frame()
        except AttributeError:
            pass
