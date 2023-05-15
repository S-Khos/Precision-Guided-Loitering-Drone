from djitellopy import Tello
import cv2, math, time, threading
import numpy as np
from pynput import keyboard
from simple_pid import PID

init_alt = 0
relative_alt = 0
spd_mag = 0
default_dist = 50

width = 960
height = 720
centreX = width // 2
centreY = height // 2
start_time = time.time()
font = cv2.FONT_HERSHEY_COMPLEX_SMALL
font_scale = .5
red = (0, 0, 255)
green = (0, 255, 0)
blue = (255, 0, 0)
white = (255, 255, 255)
black = (0, 0, 0)
line_type = 1
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
yaw_pid = [0.35, 0.3, 0.3]  #kp, ki, kd
y_pid = [0.5, 0.5, 0]
x_pid = [0.3, 0.5, 0]
z_pid = [0.4, 0.5, 0]

drone = Tello()

try:
    drone.connect()    
except:
    print("[DRONE] - Connection Error")

time.sleep(2)

init_alt = drone.get_barometer()

try:
    drone.streamon()
    frame_read = drone.get_frame_read()
except:
    print("[DRONE] - No Signal")

def flight_controller():
    global centreX, centreY, drone, yaw_pid, y_pid, x_pid, roi, tracker_ret, flt_ctrl_lock, flt_ctrl_active, tracking, manual_control
    print("[FLT CTRL] - ACTIVE")

    pid_yaw = PID(0.3,0.3,0.3,setpoint=0,output_limits=(-80,80))
    pid_y = PID(0.35,0.3,0.3,setpoint=0,output_limits=(-100,100))
    pid_x = PID(0.3,0.3,0.3,setpoint=0,output_limits=(-80,80))

    while tracking and tracker_ret and manual_control == False:
        x, y, w, h = [int(value) for value in roi]
        cur_x_error = (x + w // 2) - centreX
        cur_y_error = (y + h // 2) - centreY

        r_spd = int(pid_yaw(cur_x_error))
        x_spd = int(pid_x(cur_x_error))
        y_spd = int(pid_y(cur_y_error))

        #print("[PID]  X: {}".format(r_spd))
        if drone.send_rc_control:
            drone.send_rc_control(0, 15, y_spd, -r_spd)
        time.sleep(0.01)

    pid_yaw.reset()
    pid_y.reset()
    pid_x.reset()
    flt_ctrl_lock.acquire()
    flt_ctrl_active = False
    flt_ctrl_lock.release()
    print("[FLT CTRL] - TERMINATED")

def manual_controller(key):
    global manual_control, drone
    try:
        if key.char == 'z':
            if manual_control:
                manual_control = False
            else:
                manual_control = True
                drone.send_rc_control(0, 0, 0, 0)
    except:
        print("[MNUL CTRL] - Invalid key")

    if manual_control:
        try:
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
        time.sleep(0.01)

    track_thread_active = False
    tracker_thread = None
    tracker_lock.release()
    drone.send_rc_control(0, 0, 0, 0)
    print("[TRACK] - TRACKING TERMINATED")


def interface():
    global frame, width, height, centreX, centreY, start_time, font, font_scale, white, line_type, manual_control, empty_frame, drone

    pass



cv2.namedWindow("FEED", cv2.WINDOW_NORMAL)
cv2.moveWindow("FEED", int((1920 // 2) - (width // 2)), int(( 1080 // 2) - ( height // 2)))
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
        fps_size = cv2.getTextSize("FPS  {}".format(str(int(fps))), font, font_scale, line_type)[0][0]
        cv2.putText(frame, "FPS  {}".format(str(int(fps))), (width - fps_size - 5, 25), font, font_scale, white, line_type)

        relative_alt = (drone.get_barometer() - init_alt) * 0.0328
        spd_mag = int(math.sqrt(drone.get_speed_x() ** 2 + drone.get_speed_y() ** 2 + drone.get_speed_z() ** 2))

        # top left
        cv2.putText(frame, "BAT   {}%".format(drone.get_battery()), (5, 25), font, font_scale, white, line_type)
        cv2.putText(frame, "TEMP  {} C".format(drone.get_temperature()), (5, 55), font, font_scale, white, line_type)

        # crosshair
        cv2.line(frame, (int(width / 2) - 30, int(height / 2)), (int(width / 2) - 10, int(height / 2)), white, 2)
        cv2.line(frame, (int(width / 2) + 30, int(height / 2)), (int(width / 2) + 10, int(height / 2)), white, 2)
        cv2.line(frame, (int(width / 2), int(height / 2) - 30), (int(width / 2), int(height / 2) - 10), white, 2)
        cv2.line(frame, (int(width / 2), int(height / 2) + 30), (int(width / 2), int(height / 2) + 10), white, 2)

        #crosshair stats
        spd_size = cv2.getTextSize("SPD  {} CM/S".format(abs(spd_mag)), font, font_scale, line_type)[0][0]
        cv2.putText(frame, "SPD  {} CM/S".format(abs(spd_mag)), ((width // 2) - 90 - spd_size, (height // 2) - 100), font, font_scale, white, line_type)
        cv2.putText(frame, "ALT  {:.1f} FT".format(relative_alt), ((width // 2) + 90, (height // 2) - 100), font, font_scale, white, line_type)

        # bottom left telemtry
        cv2.putText(frame, "SPD  {}  {}  {}".format(drone.get_speed_x(), drone.get_speed_y(), drone.get_speed_z()), (5, height - 70), font, font_scale, white, line_type)
        cv2.putText(frame, "ACC  {}  {}  {}".format(drone.get_acceleration_x(), drone.get_acceleration_y(), drone.get_acceleration_z()), (5, height - 40), font, font_scale, white, line_type)
        cv2.putText(frame, "YPR  {}  {}  {}".format(drone.get_pitch(), drone.get_roll(), drone.get_height()), (5, height - 10), font, font_scale, white, line_type)

        #bottom right
        #pid_size = cv2.getTextSize("ERR  {}  {}".format(cur_x_error, cur_y_error), font, font_scale, line_type)[0][0]
        #cv2.putText(frame, "ERR  {}  {}".format(cur_x_error, cur_y_error), (width - pid_size -5, height - 40), font, #font_scale, white, line_type)

        #dist_size = cv2.getTextSize("DIST  {}  {}  {}".format(default_dist, default_dist, default_dist), font, font_scale, line_type)[0][0]
        #cv2.putText(frame, "DIST  {}  {}  {}".format(default_dist, default_dist, default_dist), (width - dist_size -5, height - 10), font, font_scale, white, line_type)

        time_size = cv2.getTextSize("T + {}".format(drone.get_flight_time()), font, font_scale, line_type)[0][0]
        cv2.putText(frame, "T + {}".format(drone.get_flight_time()), (width - time_size - 5, 55), font, font_scale, white, line_type)

        # top center
        if (manual_control and flt_ctrl_active == False):
            cv2.rectangle(frame, (width//2 - 20, 10), (width//2 + 25, 28), white, -1)
            cv2.putText(frame, "CTRL", (width//2 - 20, 25), font, font_scale, black, line_type)
        else:
            cv2.rectangle(frame, (width//2 - 20, 10), (width//2 + 30, 28), white, -1)
            cv2.putText(frame, "AUTO", (width//2 - 20, 25), font, font_scale, black, line_type)

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
            cv2.rectangle(frame, (x, y), (x + w, y + h), white, 1)
            # top
            cv2.line(frame, (x + w // 2, y), (x + w // 2, 0), white, 1)
            # left
            cv2.line(frame, (x, y + h // 2), (0, y + h // 2), white, 1)
            # right
            cv2.line(frame, (x + w, y + h // 2), (width, y + h // 2), white, 1)
            # bottom
            cv2.line(frame, (x + w // 2, y + h), (x + w // 2, height), white, 1)
                
            if flt_ctrl_active == False and manual_control == False:
                flight_ctrl_thread = threading.Thread(target=flight_controller, daemon=True)
                flight_ctrl_thread.start()
                flt_ctrl_active = True

            lock_size = cv2.getTextSize("LOCK", font, font_scale, line_type)[0][0]
            cv2.rectangle(frame, (width // 2 - (lock_size // 2), height - 38), (width // 2 + lock_size - 25, height - 20), white, -1)
            cv2.putText(frame, "LOCK", (width // 2 - (lock_size // 2), height - 22), font, font_scale, black, line_type)

        start_time = time.time()

    except Exception as error:
        print("[FEED] - Interface Error\n", error)
    try:
        cv2.imshow("FEED", frame)
    except Exception as error:
        print("[FEED] - Display Error\n", error)
        key_listener.join()
        drone.streamoff()
        drone.end()
    if (cv2.waitKey(1) & 0xff) == 27:
        break

key_listener.join()
cv2.destroyAllWindows()
drone.streamoff()
drone.end()
print("[DRONE] - CONNECTION ENDED")