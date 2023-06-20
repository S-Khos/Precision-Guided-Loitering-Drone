from djitellopy import Tello
import cv2
import time
import threading
from frontend import FrontEnd
# connect class properly, have backend create objects of everything else

drone = Tello()

frontend = FrontEnd(drone)

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
        cv2.setMouseCallback(
            "DESIGNATOR", frontend.cursor_control.event_handler)

        while frame_read:

            frame = frame_read.frame
            frame, designator_frame = frontend.update(frame)
            # frontend_thread = threading.Thread(
            #     target=frontend.update, args=(frame), daemon=True)
            # frontend_thread.start()

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
    print("[DRONE] - ", error)
    drone.end()

print("[DRONE] - CONNECTION TERMINATED")
