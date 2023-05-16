from djitellopy import Tello
import cv2, math, time, threading
import numpy as np
from pynput import keyboard
from pid import PID
import matplotlib.pyplot as plt

init_alt = 0
relative_alt = 0
spd_mag = 0
default_dist = 30

WIDTH = 960
HEIGHT = 720
CENTRE_X = WIDTH // 2
CENTRE_Y = HEIGHT // 2
start_time = time.time()
FONT = cv2.FONT_HERSHEY_COMPLEX
FONT_SCALE = .6
RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
LINE_THICKNESS = 1
manual_control = True
empty_frame = None

tracker_lock = threading.Lock()
track_thread_active = False
reset_track = False
tracker_thread = None
tracking = False
roi = None
tracker_ret = False
first_point = None
second_point = None
point_counter = 0

flt_ctrl_active = False
flt_ctrl_lock = threading.Lock()
flight_ctrl_thread = None

# tu = 1.143
# ku = 0.5 
# ki = 1.2 * 0.5 / 1.143
# kd = 3 * 0.5 * 1.143 / 40

yaw_pid = [0.30,0.52,0.042]  #kp, ki, kd

y_pid = [0.5, 0.5, 0]
x_pid = [0.3, 0.5, 0]
yaw_pid_array = []
yaw_pid_time_array = []

drone = Tello()

try:
    drone.connect()
except:
    print("[DRONE] - Connection Error")

time.sleep(2)

init_alt = drone.get_barometer()

try:
    drone.streamon()
    time.sleep(2)
    frame_read = drone.get_frame_read()
except:
    print("[DRONE] - No feed signal")

def flight_controller():
    global CENTRE_X, CENTRE_Y, drone, yaw_pid, y_pid, x_pid, roi, tracker_ret, flt_ctrl_lock, flt_ctrl_active, tracking, manual_control

    print("[FLT CTRL] - ACTIVE")

    yaw = PID(yaw_pid[0], yaw_pid[1], yaw_pid[2], CENTRE_X, -100, 100)
    y = PID(y_pid[0], y_pid[1], y_pid[2], CENTRE_Y, -100, 100)
    #x = PID(x_pid[0], x_pid[1], x_pid[2], CENTRE_X, -100, 100)

    while tracking and tracker_ret and manual_control == False:
        x, y, w, h = [int(value) for value in roi]
        targetX = x + w // 2
        targetY = y + h // 2
        yaw_spd, time_dx = yaw.compute(targetX)
        yaw_pid_array.append(yaw_spd)
        yaw_pid_time_array.append(time_dx)
        #y.compute(targetY)
        #x.compute(targetX)


        #print("[PID]  YAW: {}".format(yaw_spd))
        if drone.send_rc_control:
            drone.send_rc_control(0, 0, 0, -yaw_spd)
        time.sleep(0.01)


    yaw.reset()
    flt_ctrl_lock.acquire()
    flt_ctrl_active = False
    flt_ctrl_lock.release()
    print("[FLT CTRL] - TERMINATED")


def manual_controller(key):
    global manual_control, drone, default_dist

    try:
        if key.char == 'z':
            if manual_control:
                manual_control = False
            else:
                manual_control = True
                drone.send_rc_control(0, 0, 0, 0)

        if manual_control:
            if key.char == 'i':
                drone.takeoff()
            elif key.char == 'k':
                drone.land()
            elif key.char == 'w':
                drone.move_forward(default_dist)
            elif key.char == 'a':
                drone.move_left(default_dist)
            elif key.char == 'd':
                drone.move_right(default_dist)
            elif key.char == 's':
                drone.move_back(default_dist)
            elif key.char == 'q':
                drone.rotate_counter_clockwise(default_dist)
            elif key.char == 'e':
                drone.rotate_clockwise(default_dist)
            elif key.char == 'up':
                drone.move_up(default_dist)
            elif key.char == 'down':
                drone.move_down(default_dist)
    except:
        print("[MNUL CTRL] - Invalid key")

def on_release(key):
    if key == keyboard.Key.esc:
        return False 

def mouse_event_handler(event, x, y, flags, param):
    global tracker, tracking, point_counter, first_point, second_point, reset_track
    if event == cv2.EVENT_LBUTTONDOWN:
        if point_counter == 0:
            print("[TRACK] - P1: ({}, {})".format(x,y))
            first_point = (x, y)
            point_counter += 1
        if point_counter == 1 and (x,y) != first_point:
            print("[TRACK] - P2: ({}, {})".format(x,y))
            second_point = (x, y)
            point_counter += 1
        if tracking and point_counter == 2:
            point_counter = 0
            first_point = None
            second_point = None
            tracking = False
            reset_track = True
            print("[TRACK] - ROI RESET")
        if point_counter == 2:
            reset_track = False
            tracker = cv2.legacy.TrackerCSRT_create()
            tracker.init(empty_frame, (first_point[0], first_point[1], abs(second_point[0] - first_point[0]), abs(second_point[1] - first_point[1])))
            tracking = True

def tracker_control():
    global tracking, empty_frame, tracker, roi, tracker_ret, track_thread_active, reset_track, point_counter, drone
    tracker_lock.acquire()
    track_thread_active = True
    while tracking:
        tracker_ret, roi = tracker.update(empty_frame)
        if tracker_ret == False or reset_track:
            tracking = False
            point_counter = 0
        #time.sleep(0.01)

    track_thread_active = False
    tracker_thread = None
    tracker_lock.release()
    drone.send_rc_control(0, 0, 0, 0)
    print("[TRACK] - TRACKING TERMINATED")

cv2.namedWindow("FEED", cv2.WINDOW_NORMAL)
cv2.moveWindow("FEED", int((1920 // 2) - (WIDTH // 2)), int(( 1080 // 2) - ( HEIGHT // 2)))
cv2.setMouseCallback("FEED", mouse_event_handler)

key_listener = keyboard.Listener(on_press=manual_controller, on_release=on_release)
key_listener.start()

while True:
    try:
        frame = frame_read.frame
        timer = cv2.getTickCount()
        empty_frame = frame.copy()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # top right (fps)
        elapsed_time = time.time() - start_time
        fps = 1 / elapsed_time
        fps_size = cv2.getTextSize("FPS  {}".format(str(int(fps))), FONT, FONT_SCALE, LINE_THICKNESS)[0][0]
        cv2.putText(frame, "FPS  {}".format(str(int(fps))), (WIDTH - fps_size - 5, 25), FONT, FONT_SCALE, WHITE, LINE_THICKNESS)

        relative_alt = (drone.get_barometer() - init_alt) * 0.0328
        spd_mag = int(math.sqrt(drone.get_speed_x() ** 2 + drone.get_speed_y() ** 2 + drone.get_speed_z() ** 2))

        # top left
        cv2.putText(frame, "BAT   {}%".format(drone.get_battery()), (5, 25), FONT, FONT_SCALE, WHITE, LINE_THICKNESS)
        cv2.putText(frame, "TEMP  {} C".format(drone.get_temperature()), (5, 55), FONT, FONT_SCALE, WHITE, LINE_THICKNESS)

        # crosshair
        cv2.line(frame, (int(WIDTH / 2) - 30, int(HEIGHT / 2)), (int(WIDTH / 2) - 10, int(HEIGHT / 2)), WHITE, 2)
        cv2.line(frame, (int(WIDTH / 2) + 30, int(HEIGHT / 2)), (int(WIDTH / 2) + 10, int(HEIGHT / 2)), WHITE, 2)
        cv2.line(frame, (int(WIDTH / 2), int(HEIGHT / 2) - 30), (int(WIDTH / 2), int(HEIGHT / 2) - 10), WHITE, 2)
        cv2.line(frame, (int(WIDTH / 2), int(HEIGHT / 2) + 30), (int(WIDTH / 2), int(HEIGHT / 2) + 10), WHITE, 2)

        #crosshair stats
        spd_size = cv2.getTextSize("SPD  {} CM/S".format(abs(spd_mag)), FONT, FONT_SCALE, LINE_THICKNESS)[0][0]
        cv2.putText(frame, "SPD  {} CM/S".format(abs(spd_mag)), ((WIDTH // 2) - 90 - spd_size, (HEIGHT // 2) - 100), FONT, FONT_SCALE, WHITE, LINE_THICKNESS)
        cv2.putText(frame, "ALT  {:.1f} FT".format(relative_alt), ((WIDTH // 2) + 90, (HEIGHT // 2) - 100), FONT, FONT_SCALE, WHITE, LINE_THICKNESS)

        # bottom left telemtry
        cv2.putText(frame, "SPD  {}  {}  {}".format(drone.get_speed_x(), drone.get_speed_y(), drone.get_speed_z()), (5, HEIGHT - 70), FONT, FONT_SCALE, WHITE, LINE_THICKNESS)
        cv2.putText(frame, "ACC  {}  {}  {}".format(drone.get_acceleration_x(), drone.get_acceleration_y(), drone.get_acceleration_z()), (5, HEIGHT - 40), FONT, FONT_SCALE, WHITE, LINE_THICKNESS)
        cv2.putText(frame, "YPR  {}  {}  {}".format(drone.get_pitch(), drone.get_roll(), drone.get_HEIGHT()), (5, HEIGHT - 10), FONT, FONT_SCALE, WHITE, LINE_THICKNESS)

        time_size = cv2.getTextSize("T + {}".format(drone.get_flight_time()), FONT, FONT_SCALE, LINE_THICKNESS)[0][0]
        cv2.putText(frame, "T + {}".format(drone.get_flight_time()), (WIDTH - time_size - 5, 55), FONT, FONT_SCALE, WHITE, LINE_THICKNESS)

        # top center
        if (manual_control and flt_ctrl_active == False):
            cv2.rectangle(frame, (WIDTH//2 - 20, 10), (WIDTH//2 + 29, 28), WHITE, -1)
            cv2.putText(frame, "CTRL", (WIDTH//2 - 20, 25), FONT, FONT_SCALE, BLACK, LINE_THICKNESS)
        else:
            cv2.rectangle(frame, (WIDTH//2 - 20, 10), (WIDTH//2 + 31, 28), WHITE, -1)
            cv2.putText(frame, "AUTO", (WIDTH//2 - 20, 25), FONT, FONT_SCALE, BLACK, LINE_THICKNESS)

        if tracking and track_thread_active == False:
            print("[TRACK] - TRACKING ACTIVE")
            tracker_thread = threading.Thread(target=tracker_control, daemon=True)
            tracker_thread.start()

        if tracking == False and track_thread_active == False and tracker_thread:
            print("[TRACK] - TRACKING RESET")
            tracker_thread = None

        # active tracking / lock
        if tracker_ret and tracking:
            x, y, w, h = [int(value) for value in roi]
            cv2.rectangle(frame, (x, y), (x + w, y + h), WHITE, 1)
            # top
            cv2.line(frame, (x + w // 2, y), (x + w // 2, 0), WHITE, 1)
            # left
            cv2.line(frame, (x, y + h // 2), (0, y + h // 2), WHITE, 1)
            # right
            cv2.line(frame, (x + w, y + h // 2), (WIDTH, y + h // 2), WHITE, 1)
            # bottom
            cv2.line(frame, (x + w // 2, y + h), (x + w // 2, HEIGHT), WHITE, 1)
                
            if flt_ctrl_active == False and manual_control == False:
                flight_ctrl_thread = threading.Thread(target=flight_controller, daemon=True)
                flight_ctrl_thread.start()
                flt_ctrl_active = True

            lock_size = cv2.getTextSize("LOCK", FONT, FONT_SCALE, LINE_THICKNESS)[0][0]
            cv2.rectangle(frame, (WIDTH // 2 - (lock_size // 2), HEIGHT - 38), (WIDTH // 2 + lock_size - 25, HEIGHT - 20), WHITE, -1)
            cv2.putText(frame, "LOCK", (WIDTH // 2 - (lock_size // 2), HEIGHT - 22), FONT, FONT_SCALE, BLACK, LINE_THICKNESS)

    except Exception as error:
        print("[FEED] - Interface Error\n", error)
    try:
        cv2.imshow("FEED", frame)
    except Exception as error:
        print("[FEED] - Display Error\n", error)
        key_listener.join()
        drone.streamoff()
        drone.end()
        
    start_time = time.time()
    
    if (cv2.waitKey(1) & 0xff) == 27:
        break



key_listener.join()
cv2.destroyAllWindows()
drone.streamoff()
drone.end()

if yaw_pid_array:
    plt.plot(yaw_pid_time_array, yaw_pid_array)
    plt.show()

print("[DRONE] - CONNECTION ENDED")