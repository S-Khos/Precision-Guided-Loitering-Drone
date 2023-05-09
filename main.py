# todo
# - put tracker in seperate thread to minimize fps loss
# - implement auto flight control (homing)
# - implment manual flight control

from djitellopy import Tello
import cv2, math, time, threading
import numpy as np

init_alt = 0

width = 960
height = 720
font = cv2.FONT_HERSHEY_DUPLEX
font_scale = .6
red = (0, 0, 255)
green = (0, 255, 0)
blue = (255, 0, 0)
white = (255, 255, 255)
black = (0, 0, 0)
line_type = 1
manual_control = True

tracker = None
tracking = False
bbox = None
first_pointB = False
second_pointB = False
first_point = (0, 0)
second_point = (0, 0)


tello = Tello()
tello.connect()
tello.streamon()
init_alt = tello.get_barometer()

try:
    frame_read = tello.get_frame_read()
except:
    print("[FEED] - No Feed Signal")


# Mouse callback function
def onMouse(event, x, y, flags, param):
    global tracker, tracking, bbox

    if event == cv2.EVENT_LBUTTONDOWN:

        if tracking:
            first_pointB = False
            second_pointB = False
            tracking = False
        
        if ~first_pointB:
            first_pointB = True
            first_point = (x, y)
            
        elif ~second_pointB:
            second_pointB = True
            second_point = (x, y)

        if first_pointB and second_pointB:
            bbox = (first_point[0], first_point[1], second_point[0], second_point[1])
            tracker = cv2.legacy.TrackerCSRT_create()
            tracker.init(empty_frame, bbox)
            tracking = True

cv2.namedWindow("FEED", cv2.WINDOW_NORMAL)
cv2.setMouseCallback("FEED", onMouse)


while True:
    try:
        frame = frame_read.frame
        empty_frame = frame.copy()
        empty_frame = cv2.cvtColor(empty_frame, cv2.COLOR_BGR2RBG)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # top left
        cv2.putText(frame, "BAT   {}%".format(tello.get_battery()), (5, 25), font, font_scale, white, line_type)
        cv2.putText(frame, "TEMP  {} C".format(tello.get_temperature()), (5, 55), font, font_scale, white, line_type)

        # crosshair
        cv2.line(frame, (int(width / 2) - 30, int(height / 2)), (int(width / 2) - 10, int(height / 2)), white, 1)
        cv2.line(frame, (int(width / 2) + 30, int(height / 2)), (int(width / 2) + 10, int(height / 2)), white, 1)
        cv2.line(frame, (int(width / 2), int(height / 2) - 30), (int(width / 2), int(height / 2) - 10), white, 1)
        cv2.line(frame, (int(width / 2), int(height / 2) + 30), (int(width / 2), int(height / 2) + 10), white, 1)

        #crosshair stats
        spd_size = cv2.getTextSize("SPD  0", font, font_scale, line_type)[0][0]
        cv2.putText(frame, "SPD  {}".format(abs(tello.get_speed_x())), ((width // 2) - 90 - spd_size, (height // 2) - 100), font, font_scale, white, line_type)
        cv2.putText(frame, "ALT  {:.1f}".format((tello.get_barometer() - init_alt) * 0.0328), ((width // 2) + 90, (height // 2) - 100), font, font_scale, white, line_type)

        # bottom left telemtry
        cv2.putText(frame, "VRT SPD  {}".format(tello.get_speed_z()), (5, height - 70), font, font_scale, white, line_type)
        cv2.putText(frame, "HRZ SPD  {}".format(tello.get_speed_y()), (5, height - 40), font, font_scale, white, line_type)
        cv2.putText(frame, "YPR  {}  {}  {}".format(tello.get_pitch(), tello.get_roll(), tello.get_height()), (5, height - 10), font, font_scale, white, line_type)

        # top center
        if (manual_control):
            
            cv2.rectangle(frame, (width//2 - 20, 10), (width//2 + 30, 28), white, -1)
            cv2.putText(frame, "MANL", (width//2 - 20, 25), font, font_scale, black, line_type)

        else:

            cv2.rectangle(frame, (width//2 - 20, 10), (width//2 + 30, 28), white, -1)
            cv2.putText(frame, "AUTO", (width//2 - 20, 25), font, font_scale, black, line_type)

        if tracking:
            lock_size = cv2.getTextSize("LOCK", font, font_scale, line_type)[0][0]
            cv2.rectangle(frame, (width//2 - (lock_size // 2), height - 38), (width//2 + lock_size - 25, height - 20), white, -1)
            cv2.putText(frame, "LOCK", (width//2 - (lock_size // 2), height - 23), font, font_scale, black, line_type)


        if tracking:
            ret, bbox = tracker.update(empty_frame)

            if ret:
                # Draw the tracked object on the frame
                x, y, w, h = [int(value) for value in bbox]
                # top left
                cv2.line(frame, (x, y), (x + 50, y), white, 1)
                cv2.line(frame, (x, y), (x, y + 50), white, 1)
                # top right
                cv2.line(frame, (x + w, y), (x + w - 50, y), white, 1)
                cv2.line(frame, (x + w, y), (x + w, y + 50), white, 1)
                # bottom left
                cv2.line(frame, (x, y + h), (x + 50, y + h), white, 1)
                cv2.line(frame, (x, y + h), (x, y + h - 50), white, 1)
                # bottom right
                cv2.line(frame, (x + w, y + h), (x + w - 50, y + h), white, 1)
                cv2.line(frame, (x + w, y + h), (x + w, y + h - 50), white, 1)

            else:

                tracking = False




    except:
        print("[FEED] - UI Error")

    try:
        cv2.imshow("FEED", frame)
    except:
        print("[FEED] - Frame Display Error")

    if (cv2.waitKey(1) & 0xff) == 27:
        break

cv2.destroyAllWindows()
tello.end()
