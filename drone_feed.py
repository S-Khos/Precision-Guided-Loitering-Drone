import cv2
import time
from threading import Thread
from tello import Tello


class Drone(Object):
    def __init__(self, drone):
        drone.streamon()
        self.feed_signal = drone.get_frame_read()
        self.feed = None

    def update_feed(self, drone):
        while self.feed_signal:
            self.feed = self.feed_signal.frame

        try:
            cv2.imshow("FEED", frame)
        except Exception as error:
            print("[FEED] - Display error\n", error)
            key_listener.join()
            drone.streamoff()
            drone.end()
            break

    start_time = time.time()
    if (cv2.waitKey(1) & 0xff) == 27:
        break

    def get_feed(self):
        return self.feed
