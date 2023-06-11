from djitellopy import Tello
import cv2
import math
import time
import threading
import numpy as np
from pynput import keyboard
from pid import PID
import matplotlib.pyplot as plt
from manual_control import KeyControl, CursorControl
from tracker import Tracker

init_alt = 0
relative_alt = 0
altitude = 0
spd_mag = 0
default_dist = 30

WIDTH = 960
HEIGHT = 720
CENTRE_X = WIDTH // 2
CENTRE_Y = HEIGHT // 2
FONT = cv2.FONT_HERSHEY_COMPLEX
FONT_SCALE = .6
RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
ui_text_clr = GREEN
LINE_THICKNESS = 1
empty_frame = None

tracker_lock = threading.Lock()
track_thread_active = False
reset_track = True
tracker_thread = None
tracking = False
roi = None
tracker_ret = False
lock = False
roi_size = [100, 100]
cursor_pos = [CENTRE_X, CENTRE_Y]

flt_ctrl_active = False
flt_ctrl_lock = threading.Lock()
flight_ctrl_thread = None

# add derivative to reduce overshoot, add integral to reduce steady state error, add proportional to reduce rise time

YAW_PID = [0.32, 0.05, 0.11]  # 0.32, 0, 0.06
Y_PID = [1.3, 0.18, 0.1]  # 0.1, 0.3, 0.3,
X_PID = [0.2, 0.0, 0.12]
yaw_pid_array = []
yaw_pid_time = []
drone = Tello()

manual_control = KeyControl(drone)
# cursor_control = CursorControl(manual_control)
# tracker = Tracker(cursor_control, manual_control)


try:
    drone.connect()
    if (drone.get_battery() <= 20):
        print("[DRONE] - Low battery")
        drone.end()
except:
    print("[DRONE] - Connection Error")

time.sleep(0.5)

try:
    drone.streamon()
    time.sleep(0.5)
    frame_read = drone.get_frame_read()
except:
    print("[DRONE] - No feed signal")


def guidance_system():
    global CENTRE_X, CENTRE_Y, drone, yaw_pid, y_pid, x_pid, roi, tracker_ret, flt_ctrl_lock, flt_ctrl_active, tracking, manual_control, lock, altitude

    try:
        print("[FLT CTRL] - ACTIVE")

        yaw_pid = PID(YAW_PID[0], YAW_PID[1], YAW_PID[2], CENTRE_X, -100, 100)
        x_pid = PID(X_PID[0], X_PID[1], X_PID[2], CENTRE_X, -80, 80)
        y_pid = PID(Y_PID[0], Y_PID[1], Y_PID[2], CENTRE_Y, -100, 100)

        while tracking and tracker_ret and not manual_control.manual:
            x, y, w, h = [int(value) for value in roi]
            targetX = x + w // 2
            targetY = y + h // 2

            yaw_velocity, yaw_time = yaw_pid.update(targetX)
            x_velocity, x_time = x_pid.update(targetX)
            y_velocity, y_time = y_pid.update(targetY)

            yaw_pid_array.append(yaw_velocity)
            yaw_pid_time.append(yaw_time)

            if drone.send_rc_control:
                drone.send_rc_control(-x_velocity if abs(x_velocity)
                                      > 60 else 0, 90 if altitude > 1 and manual_control.dive else 0, y_velocity, -yaw_velocity)

            time.sleep(0.01)

        yaw_pid.reset()
        y_pid.reset()
        x_pid.reset()
        flt_ctrl_lock.acquire()
        flt_ctrl_active = False
        flt_ctrl_lock.release()
        drone.send_rc_control(0, 0, 0, 0)
        print("[FLT CTRL] - TERMINATED")

    except Exception as error:
        yaw_pid.reset()
        y_pid.reset()
        x_pid.reset()
        flt_ctrl_lock.acquire()
        flt_ctrl_active = False
        manual_control.flip_manual()
        flt_ctrl_lock.release()
        drone.send_rc_control(0, 0, 0, 0)
        print("[FLT CTRL] - Error occured\n", error)


def mouse_event_handler(event, x, y, flags, param):
    global tracker, tracking, reset_track, cursor_pos, manual_control
    cursor_pos[0] = x - manual_control.designator_roi_size[0] // 2
    cursor_pos[1] = y - manual_control.designator_roi_size[1] // 2
    if event == cv2.EVENT_LBUTTONDOWN:
        if (not tracking and reset_track):
            reset_track = False
            tracker = cv2.legacy.TrackerCSRT_create()
            tracker.init(
                empty_frame, (cursor_pos[0], cursor_pos[1], manual_control.designator_roi_size[0], manual_control.designator_roi_size[1]))
            tracking = True
        else:
            reset_track = True


def tracker_control():
    global tracking, empty_frame, tracker, roi, tracker_ret, track_thread_active, reset_track, drone
    tracker_lock.acquire()
    track_thread_active = True
    print("[TRACK] - TRACKING ACTIVE")
    try:
        while tracking:
            tracker_ret, roi = tracker.update(empty_frame)
            if tracker_ret == False or reset_track == True:
                tracking = False
                reset_track = True

    except:
        print("[TRACK] - Invalid Coordinates")
        tracking = False
        reset_track = True

    track_thread_active = False
    tracker_thread = None
    tracker_lock.release()
    drone.send_rc_control(0, 0, 0, 0)
    print("[TRACK] - TRACKING TERMINATED")


cv2.namedWindow("FEED", cv2.WINDOW_NORMAL)
cv2.namedWindow("DESIGNATOR", cv2.WINDOW_NORMAL)
cv2.moveWindow("FEED", int((1920 // 4) - (WIDTH // 2)),
               int((1080 // 2) - (HEIGHT // 2)))
cv2.moveWindow("DESIGNATOR", int((1920 // 4) + (WIDTH // 2)),
               int((1080 // 2) - (HEIGHT // 2)))

cv2.setMouseCallback("DESIGNATOR", mouse_event_handler)

start_time = time.time()

while frame_read:
    try:
        frame = frame_read.frame
        timer = cv2.getTickCount()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        empty_frame = frame.copy()

        # top right (fps)
        elapsed_time = time.time() - start_time
        fps = 1 / elapsed_time
        fps_size = cv2.getTextSize("FPS  {}".format(
            str(int(fps))), FONT, FONT_SCALE, LINE_THICKNESS)[0][0]
        cv2.putText(frame, "FPS  {}".format(str(int(fps))), (WIDTH -
                    fps_size - 5, 25), FONT, FONT_SCALE, ui_text_clr, LINE_THICKNESS)

        altitude = drone.get_distance_tof() / 30.48
        spd_mag = int(math.sqrt(drone.get_speed_x() ** 2 +
                      drone.get_speed_y() ** 2 + drone.get_speed_z() ** 2))

        # top left
        cv2.putText(frame, "PWR   {}%".format(drone.get_battery()),
                    (5, 25), FONT, FONT_SCALE, ui_text_clr, LINE_THICKNESS)
        cv2.putText(frame, "TMP  {} C".format(drone.get_temperature()),
                    (5, 55), FONT, FONT_SCALE, ui_text_clr, LINE_THICKNESS)

        # crosshair
        cv2.line(frame, (int(WIDTH / 2) - 20, int(HEIGHT / 2)),
                 (int(WIDTH / 2) - 10, int(HEIGHT / 2)), ui_text_clr, 2)
        cv2.line(frame, (int(WIDTH / 2) + 20, int(HEIGHT / 2)),
                 (int(WIDTH / 2) + 10, int(HEIGHT / 2)), ui_text_clr, 2)
        cv2.line(frame, (int(WIDTH / 2), int(HEIGHT / 2) - 20),
                 (int(WIDTH / 2), int(HEIGHT / 2) - 10), ui_text_clr, 2)
        cv2.line(frame, (int(WIDTH / 2), int(HEIGHT / 2) + 20),
                 (int(WIDTH / 2), int(HEIGHT / 2) + 10), ui_text_clr, 2)

        # crosshair stats
        spd_size = cv2.getTextSize(
            "SPD  {} CM/S".format(abs(spd_mag)), FONT, FONT_SCALE, LINE_THICKNESS)[0][0]
        cv2.putText(frame, "SPD  {} CM/S".format(abs(spd_mag)), ((WIDTH // 2) - 90 -
                    spd_size, (HEIGHT // 2) - 100), FONT, FONT_SCALE, ui_text_clr, LINE_THICKNESS)
        cv2.putText(frame, "ALT  {:.1f} FT".format(drone.get_distance_tof() / 30.48), ((
            WIDTH // 2) + 90, (HEIGHT // 2) - 100), FONT, FONT_SCALE, ui_text_clr, LINE_THICKNESS)

        # bottom left telemtry
        cv2.putText(frame, "BRM  {}".format(int(drone.get_barometer() / 30.48)),
                    (5, HEIGHT - 100), FONT, FONT_SCALE, ui_text_clr, LINE_THICKNESS)
        cv2.putText(frame, "SPD  {}  {}  {}".format(drone.get_speed_x(), drone.get_speed_y(
        ), drone.get_speed_z()), (5, HEIGHT - 70), FONT, FONT_SCALE, ui_text_clr, LINE_THICKNESS)
        cv2.putText(frame, "ACC  {}  {}  {}".format(int(drone.get_acceleration_x()), int(drone.get_acceleration_y(
        )), int(drone.get_acceleration_z())), (5, HEIGHT - 40), FONT, FONT_SCALE, ui_text_clr, LINE_THICKNESS)
        cv2.putText(frame, "YPR  {}  {}  {}".format(drone.get_yaw(), drone.get_pitch(
        ), drone.get_roll()), (5, HEIGHT - 10), FONT, FONT_SCALE, ui_text_clr, LINE_THICKNESS)

        time_size = cv2.getTextSize(
            "T + {}".format(drone.get_flight_time()), FONT, FONT_SCALE, LINE_THICKNESS)[0][0]
        cv2.putText(frame, "T + {}".format(drone.get_flight_time()),
                    (WIDTH - time_size - 5, 55), FONT, FONT_SCALE, ui_text_clr, LINE_THICKNESS)

        # bottom compass
        cv2.circle(frame, (WIDTH - 60, HEIGHT - 60), 50, ui_text_clr, 1)
        cv2.arrowedLine(frame, (WIDTH - 60, HEIGHT - 60), (int(-50 * math.cos(math.radians(drone.get_yaw() + 90)) +
                        WIDTH - 60), int((HEIGHT - 60) - (50 * math.sin(math.radians(drone.get_yaw() + 90))))), ui_text_clr, 1, tipLength=.15)

        # top center
        if (manual_control.manual and not flt_ctrl_active):

            cv2.rectangle(frame, (WIDTH//2 - 20, 10),
                          (WIDTH//2 + 29, 28), ui_text_clr, -1)
            cv2.putText(frame, "CTRL", (WIDTH//2 - 20, 25),
                        FONT, FONT_SCALE, BLACK, LINE_THICKNESS)
        else:
            cv2.rectangle(frame, (WIDTH//2 - 20, 10),
                          (WIDTH//2 + 31, 28), ui_text_clr, -1)
            cv2.putText(frame, "AUTO", (WIDTH//2 - 20, 25),
                        FONT, FONT_SCALE, BLACK, LINE_THICKNESS)

        if tracking and track_thread_active == False:
            tracker_thread = threading.Thread(
                target=tracker_control, daemon=True)
            tracker_thread.start()

        if tracking == False and track_thread_active == False and tracker_thread:
            print("[TRACK] - TRACKING RESET")
            tracker_thread = None

        if not tracking:
            cv2.rectangle(empty_frame, (cursor_pos[0], cursor_pos[1]), (
                cursor_pos[0] + manual_control.designator_roi_size[0], cursor_pos[1] + manual_control.designator_roi_size[1]), ui_text_clr, 1)
            # top
            cv2.line(empty_frame, (cursor_pos[0] + manual_control.designator_roi_size[0] // 2, cursor_pos[1]),
                     (cursor_pos[0] + manual_control.designator_roi_size[0] // 2, 0), ui_text_clr, 1)
            # left
            cv2.line(empty_frame, (cursor_pos[0], cursor_pos[1] + manual_control.designator_roi_size[1] // 2),
                     (0, cursor_pos[1] + manual_control.designator_roi_size[1] // 2), ui_text_clr, 1)
            # right
            cv2.line(empty_frame, (cursor_pos[0] + manual_control.designator_roi_size[0], cursor_pos[1] + manual_control.designator_roi_size[1] // 2),
                     (WIDTH, cursor_pos[1] + manual_control.designator_roi_size[1] // 2), WHITE, 1)
            # bottom
            cv2.line(empty_frame, (cursor_pos[0] + manual_control.designator_roi_size[0] // 2, cursor_pos[1] + manual_control.designator_roi_size[1]),
                     (cursor_pos[0] + manual_control.designator_roi_size[0] // 2, HEIGHT), WHITE, 1)

        # active tracking / lock
        if tracker_ret and tracking:
            x, y, w, h = [int(value) for value in roi]
            if (CENTRE_X > x and CENTRE_X < x + w and CENTRE_Y > y and CENTRE_Y < y + h and not manual_control.manual):
                lock = True
                lock_size = cv2.getTextSize(
                    "LOCK", FONT, FONT_SCALE, LINE_THICKNESS)[0][0]
                cv2.rectangle(frame, (WIDTH // 2 - (lock_size // 2), HEIGHT - 38),
                              (WIDTH // 2 + lock_size - 25, HEIGHT - 20), ui_text_clr, -1)
                cv2.putText(frame, "LOCK", (WIDTH // 2 - (lock_size // 2),
                            HEIGHT - 22), FONT, FONT_SCALE, BLACK, LINE_THICKNESS)
            else:
                lock = False
                trk_size = cv2.getTextSize(
                    "TRK", FONT, FONT_SCALE, LINE_THICKNESS)[0][0]
                cv2.rectangle(frame, (WIDTH // 2 - (trk_size // 2), HEIGHT - 38),
                              (WIDTH // 2 + trk_size - 20, HEIGHT - 20), ui_text_clr, -1)
                cv2.putText(frame, "TRK", (WIDTH // 2 - (trk_size // 2),
                            HEIGHT - 22), FONT, FONT_SCALE, BLACK, LINE_THICKNESS)

            cv2.line(frame, (x, y), (x + 20, y),
                     RED if manual_control.dive else ui_text_clr, 2)
            cv2.line(frame, (x, y), (x, y + 20),
                     RED if manual_control.dive else ui_text_clr, 2)
            cv2.line(frame, (x, y + h), (x, y + h - 20),
                     RED if manual_control.dive else ui_text_clr, 2)
            cv2.line(frame, (x, y + h), (x + 20, y + h),
                     RED if manual_control.dive else ui_text_clr, 2)

            cv2.line(frame, (x + w, y), (x + w - 20, y),
                     RED if manual_control.dive else ui_text_clr, 2)
            cv2.line(frame, (x + w, y), (x + w, y + 20),
                     RED if manual_control.dive else ui_text_clr, 2)
            cv2.line(frame, (x + w, y + h), (x + w, y + h - 20),
                     RED if manual_control.dive else ui_text_clr, 2)
            cv2.line(frame, (x + w, y + h), (x + w - 20, y + h),
                     RED if manual_control.dive else ui_text_clr, 2)

            cv2.circle(frame, (x + w // 2, y + h // 2), 3,
                       RED if manual_control.dive else ui_text_clr, -1)
            # top
            cv2.line(frame, (x + w // 2, y), (x + w // 2, 0), ui_text_clr, 1)
            # left
            cv2.line(frame, (x, y + h // 2), (0, y + h // 2), ui_text_clr, 1)
            # right
            cv2.line(frame, (x + w, y + h // 2),
                     (WIDTH, y + h // 2), ui_text_clr, 1)
            # bottom
            cv2.line(frame, (x + w // 2, y + h),
                     (x + w // 2, HEIGHT), ui_text_clr, 1)

            if not flt_ctrl_active and manual_control.manual:
                flight_ctrl_thread = threading.Thread(
                    target=guidance_system, daemon=True)
                flight_ctrl_thread.start()
                flt_ctrl_active = True

    except Exception as error:
        print("[FEED] - Interface error\n", error)
    try:
        cv2.imshow("FEED", frame)
        cv2.imshow("DESIGNATOR", empty_frame)

    except Exception as error:
        print("[FEED] - Display error\n", error)
        drone.streamoff()
        drone.end()
        break

    start_time = time.time()
    if (cv2.waitKey(1) & 0xff) == 27:
        break

cv2.destroyAllWindows()
drone.streamoff()
drone.end()
print("[DRONE] - CONNECTION TERMINATED")

if yaw_pid_array:
    plt.plot(yaw_pid_time, yaw_pid_array)
    plt.show()
