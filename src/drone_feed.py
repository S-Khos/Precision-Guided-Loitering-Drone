import cv2
import time
import threading
from djitellopy import Tello

drone = Tello()
drone.connect()
drone.streamon()
frame_read = drone.get_frame_read(with_queue=True)


cv2.namedWindow("FEED", cv2.WINDOW_NORMAL)
while True:
    frame = frame_read.frame
    cv2.imshow("FEED", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
drone.end()
