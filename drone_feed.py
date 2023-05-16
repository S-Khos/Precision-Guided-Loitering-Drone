import cv2, math, time, threading

class DroneFeed(object):
    def __init__(self, drone):
        self.drone = drone
        self.frame_read = self.drone.get_frame_read()
    
    def display_frame(self):
        cv2.namedWindow("FEED", cv2.WINDOW_NORMAL)
        cv2.moveWindow("FEED", int((1920 // 2) - (WIDTH // 2)), int(( 1080 // 2) - ( HEIGHT // 2)))
        while True:
            frame = self.frame_read.frame
            try:
                cv2.imshow("FEED", frame)
            except Exception as error:
                print("[FEED] - Display Error\n", error)
                self.drone.streamoff()
                self.drone.end()
    
            if (cv2.waitKey(1) & 0xff) == 27:
                break
