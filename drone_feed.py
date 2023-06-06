import cv2
import time
from threading import Thread
from djitellopy import Tello
import queue

feed_queue = queue.Queue()
drone = Tello()
drone.connect()
drone.streamon()
frame_read = drone.get_frame_read()


def receive():
    global drone, frame_read, feed_queue
    while True:
        frame = frame_read.frame
        feed_queue.put(frame)


def display():
    global feed_queue
    while True:
        if not feed_queue.empty():
            frame = feed_queue.get()
            cv2.imshow('Drone Feed', frame)
            cv2.waitKey(1)


thread1 = Thread(target=receive)
thread2 = Thread(target=display)
thread1.start()
thread2.start()
