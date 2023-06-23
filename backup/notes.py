# if event == cv2.EVENT_LBUTTONDOWN:
#     if point_counter == 0:
#         # print("[TRACK] - P1: ({}, {})".format(x,y))
#         first_point = (x, y)
#         point_counter += 1
#     if point_counter == 1 and (x, y) != first_point:
#         # print("[TRACK] - P2: ({}, {})".format(x,y))
#         second_point = (x, y)
#         point_counter += 1
#     if tracking and point_counter == 2:
#         point_counter = 0
#         first_point = None
#         second_point = None
#         tracking = False
#         reset_track = True
#         # print("[TRACK] - ROI RESET")
#     if point_counter == 2:
#         reset_track = False
#         tracker = cv2.legacy.TrackerCSRT_create()
#         tracker.init(empty_frame, (first_point[0], first_point[1], abs(
#             second_point[0] - first_point[0]), abs(second_point[1] - first_point[1])))
#         tracking = True


# def mouse_event_handler(event, x, y, flags, param):
#     global tracker, tracking, reset_track, cursor_pos, manual_control
#     cursor_pos[0] = x - manual_control.designator_roi_size[0] // 2
#     cursor_pos[1] = y - manual_control.designator_roi_size[1] // 2
#     if event == cv2.EVENT_LBUTTONDOWN:
#         if (not tracking and reset_track):
#             reset_track = False
#             tracker = cv2.legacy.TrackerCSRT_create()
#             tracker.init(
#                 empty_frame, (cursor_pos[0], cursor_pos[1], manual_control.designator_roi_size[0], manual_control.designator_roi_size[1]))
#             tracking = True
#         else:
#             reset_track = True


def tracker_control():
    global empty_frame, roi, tracker_ret, track_thread_active, drone, cursor_control
    tracker_lock.acquire()
    track_thread_active = True
    print("[TRACK] - TRACKING ACTIVE")
    try:
        while tracker.tracking:
            tracker_ret, roi = tracker.update(empty_frame)
            if not tracker_ret or cursor_control.reset_track:
                cursor_control.tracking = False
                cursor_control.reset_track = True

    except:
        print("[TRACK] - Invalid Coordinates")
        cursor_control.tracking = False
        cursor_control.reset_track = True

    track_thread_active = False
    tracker_thread = None
    tracker_lock.release()
    drone.send_rc_control(0, 0, 0, 0)
    print("[TRACK] - TRACKING TERMINATED")


def guidance_system():
    global drone, yaw_pid, y_pid, x_pid, roi, tracker_ret, flt_ctrl_lock, flt_ctrl_active, manual_control, cursor_control, lock, altitude, frontend

    try:
        print("[FLT CTRL] - ACTIVE")

        yaw_pid = PID(YAW_PID[0], YAW_PID[1], YAW_PID[2],
                      frontend.CENTRE_X, -100, 100)
        x_pid = PID(X_PID[0], X_PID[1], X_PID[2], frontend.CENTRE_X, -80, 80)
        y_pid = PID(Y_PID[0], Y_PID[1], Y_PID[2], frontend.CENTRE_Y, -100, 100)

        while cursor_control.tracking and tracker_ret and not manual_control.manual:
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

            time.sleep(0.1)

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


def main():

    drone = Tello()
    state = State(drone)
    backend = BackEnd(state)
    frontend = FrontEnd(state)

    drone.connect()
    if (drone.get_battery() > 25):
        drone.streamon()
        frame_read = drone.get_frame_read()
        cv2.namedWindow("FEED", cv2.WINDOW_NORMAL)
        cv2.namedWindow("DESIGNATOR", cv2.WINDOW_NORMAL)
        cv2.moveWindow("FEED", int((1920 // 4) - state.CENTRE_X),
                       int((1080 // 2) - state.CENTRE_Y))
        cv2.moveWindow("DESIGNATOR", int((1920 // 4) + state.CENTRE_X + 3),
                       int((1080 // 2) - state.CENTRE_Y))

        cv2.setMouseCallback(
            "DESIGNATOR", backend.cursor_control.event_handler)

        while frame_read:
            state.frame = frame_read.frame

            state.update()
            backend.update()
            frontend.update()

            cv2.imshow("FEED", state.frame)
            cv2.imshow("DESIGNATOR", state.designator_frame)
            if (cv2.waitKey(1) & 0xff) == 27:
                break

        cv2.destroyAllWindows()
        drone.streamoff()
        drone.end()

    else:
        print("[DRONE] - Low battery")
        drone.end()

    drone.end()

    print("[DRONE] - CONNECTION TERMINATED")
