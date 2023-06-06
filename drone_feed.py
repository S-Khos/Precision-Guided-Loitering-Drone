import cv2
import time
import threading
from djitellopy import Tello
import queue

feed_queue = queue.Queue()
drone = Tello()
drone.connect()
drone.streamon()
frame_read = drone.get_frame_read()
WIDTH = 960
HEIGHT = 720


def receive():
    global frame_read, feed_queue
    while True:
        frame = frame_read.frame
        feed_queue.put(frame)


cv2.namedWindow("FEED", cv2.WINDOW_NORMAL)
cv2.moveWindow("FEED", int((1920 // 2) - (WIDTH // 2)),
               int((1080 // 2) - (HEIGHT // 2)))


def display():
    global feed_queue
    while True:
        if not feed_queue.empty():
            frame = feed_queue.get()
            cv2.imshow("FEED", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


thread1 = threading.Thread(target=receive, daemon=True)
thread2 = threading.Thread(target=display)
thread1.start()
thread2.start()
thread2join()
