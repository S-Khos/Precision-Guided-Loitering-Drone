from djitellopy import Tello
import cv2
import math
import time
import threading
from pynput import keyboard
from pid import PID
import matplotlib.pyplot as plt
from manual_control import KeyControl, CursorControl
from frontend import FrontEnd
from tracker import Tracker
from backend import Backend

drone = Tello()
frontend = FrontEnd(drone)
manual_control = KeyControl(drone)
cursor_control = CursorControl(manual_control, frontend)
tracker = Tracker(cursor_control, manual_control, frontend)

try:
    drone.connect()
    if (drone.get_battery() > 25):
        drone.streamon()
        frame_read = drone.get_frame_read()
        cv2.namedWindow("FEED", cv2.WINDOW_NORMAL)
        cv2.namedWindow("DESIGNATOR", cv2.WINDOW_NORMAL)
        cv2.moveWindow("FEED", int((1920 // 4) - (frontend.FRAME_WIDTH // 2)),
                       int((1080 // 2) - (frontend.FRAME_HEIGHT // 2)))
        cv2.moveWindow("DESIGNATOR", int((1920 // 4) + (frontend.FRAME_WIDTH // 2) + 10),
                       int((1080 // 2) - (frontend.FRAME_HEIGHT // 2)))
        cv2.setMouseCallback("DESIGNATOR", cursor_control.event_handler)
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
                cv2.putText(frame, "FPS  {}".format(str(int(fps))), (frontend.FRAME_WIDTH -
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
                cv2.line(frame, (int(frontend.FRAME_WIDTH / 2) - 20, int(frontend.FRAME_HEIGHT / 2)),
                         (int(frontend.FRAME_WIDTH / 2) - 10, int(frontend.FRAME_HEIGHT / 2)), ui_text_clr, 2)
                cv2.line(frame, (int(frontend.FRAME_WIDTH / 2) + 20, int(frontend.FRAME_HEIGHT / 2)),
                         (int(frontend.FRAME_WIDTH / 2) + 10, int(frontend.FRAME_HEIGHT / 2)), ui_text_clr, 2)
                cv2.line(frame, (int(frontend.FRAME_WIDTH / 2), int(frontend.FRAME_HEIGHT / 2) - 20),
                         (int(frontend.FRAME_WIDTH / 2), int(frontend.FRAME_HEIGHT / 2) - 10), ui_text_clr, 2)
                cv2.line(frame, (int(frontend.FRAME_WIDTH / 2), int(frontend.FRAME_HEIGHT / 2) + 20),
                         (int(frontend.FRAME_WIDTH / 2), int(frontend.FRAME_HEIGHT / 2) + 10), ui_text_clr, 2)

                # crosshair stats
                spd_size = cv2.getTextSize(
                    "SPD  {} CM/S".format(abs(spd_mag)), FONT, FONT_SCALE, LINE_THICKNESS)[0][0]
                cv2.putText(frame, "SPD  {} CM/S".format(abs(spd_mag)), ((frontend.FRAME_WIDTH // 2) - 90 -
                            spd_size, (frontend.FRAME_HEIGHT // 2) - 100), FONT, FONT_SCALE, ui_text_clr, LINE_THICKNESS)
                cv2.putText(frame, "ALT  {:.1f} FT".format(drone.get_distance_tof() / 30.48), ((
                    frontend.FRAME_WIDTH // 2) + 90, (frontend.FRAME_HEIGHT // 2) - 100), FONT, FONT_SCALE, ui_text_clr, LINE_THICKNESS)

                # bottom left telemtry
                cv2.putText(frame, "BRM  {}".format(int(drone.get_barometer() / 30.48)),
                            (5, frontend.FRAME_HEIGHT - 100), FONT, FONT_SCALE, ui_text_clr, LINE_THICKNESS)
                cv2.putText(frame, "SPD  {}  {}  {}".format(drone.get_speed_x(), drone.get_speed_y(
                ), drone.get_speed_z()), (5, frontend.FRAME_HEIGHT - 70), FONT, FONT_SCALE, ui_text_clr, LINE_THICKNESS)
                cv2.putText(frame, "ACC  {}  {}  {}".format(int(drone.get_acceleration_x()), int(drone.get_acceleration_y(
                )), int(drone.get_acceleration_z())), (5, frontend.FRAME_HEIGHT - 40), FONT, FONT_SCALE, ui_text_clr, LINE_THICKNESS)
                cv2.putText(frame, "YPR  {}  {}  {}".format(drone.get_yaw(), drone.get_pitch(
                ), drone.get_roll()), (5, frontend.FRAME_HEIGHT - 10), FONT, FONT_SCALE, ui_text_clr, LINE_THICKNESS)

                time_size = cv2.getTextSize(
                    "T + {}".format(drone.get_flight_time()), FONT, FONT_SCALE, LINE_THICKNESS)[0][0]
                cv2.putText(frame, "T + {}".format(drone.get_flight_time()),
                            (frontend.FRAME_WIDTH - time_size - 5, 55), FONT, FONT_SCALE, ui_text_clr, LINE_THICKNESS)

                # bottom compass
                cv2.circle(frame, (frontend.FRAME_WIDTH - 60, frontend.FRAME_HEIGHT - 60),
                           50, ui_text_clr, 1)
                cv2.arrowedLine(frame, (frontend.FRAME_WIDTH - 60, frontend.FRAME_HEIGHT - 60), (int(-50 * math.cos(math.radians(drone.get_yaw() + 90)) +
                                frontend.FRAME_WIDTH - 60), int((frontend.FRAME_HEIGHT - 60) - (50 * math.sin(math.radians(drone.get_yaw() + 90))))), ui_text_clr, 1, tipLength=.15)

                # top center
                if (manual_control.manual and not flt_ctrl_active):

                    cv2.rectangle(frame, (frontend.FRAME_WIDTH//2 - 20, 10),
                                  (frontend.FRAME_WIDTH//2 + 29, 28), ui_text_clr, -1)
                    cv2.putText(frame, "CTRL", (frontend.FRAME_WIDTH//2 - 20, 25),
                                FONT, FONT_SCALE, BLACK, LINE_THICKNESS)
                else:
                    cv2.rectangle(frame, (frontend.FRAME_WIDTH//2 - 20, 10),
                                  (frontend.FRAME_WIDTH//2 + 31, 28), ui_text_clr, -1)
                    cv2.putText(frame, "AUTO", (frontend.FRAME_WIDTH//2 - 20, 25),
                                FONT, FONT_SCALE, BLACK, LINE_THICKNESS)

                if not cursor_control.tracking and not track_thread_active and tracker_thread:
                    print("[TRACK] - TRACKING RESET")
                    tracker_thread = None

                if not cursor_control.tracking:
                    cv2.rectangle(empty_frame, (cursor_control.cursor_pos[0], cursor_control.cursor_pos[1]), (
                        cursor_control.cursor_pos[0] + manual_control.designator_roi_size[0], cursor_control.cursor_pos[1] + manual_control.designator_roi_size[1]), ui_text_clr, 1)
                    # top
                    cv2.line(empty_frame, (cursor_control.cursor_pos[0] + manual_control.designator_roi_size[0] // 2, cursor_control.cursor_pos[1]),
                             (cursor_control.cursor_pos[0] + manual_control.designator_roi_size[0] // 2, 0), ui_text_clr, 1)
                    # left
                    cv2.line(empty_frame, (cursor_control.cursor_pos[0], cursor_control.cursor_pos[1] + manual_control.designator_roi_size[1] // 2),
                             (0, cursor_control.cursor_pos[1] + manual_control.designator_roi_size[1] // 2), ui_text_clr, 1)
                    # right
                    cv2.line(empty_frame, (cursor_control.cursor_pos[0] + manual_control.designator_roi_size[0], cursor_control.cursor_pos[1] + manual_control.designator_roi_size[1] // 2),
                             (frontend.FRAME_WIDTH, cursor_control.cursor_pos[1] + manual_control.designator_roi_size[1] // 2), ui_text_clr, 1)
                    # bottom
                    cv2.line(empty_frame, (cursor_control.cursor_pos[0] + manual_control.designator_roi_size[0] // 2, cursor_control.cursor_pos[1] + manual_control.designator_roi_size[1]),
                             (cursor_control.cursor_pos[0] + manual_control.designator_roi_size[0] // 2, frontend.FRAME_HEIGHT), ui_text_clr, 1)

                # active tracking / lock
                if tracker.tracking:
                    x, y, w, h = [int(value) for value in tracker.bbox]
                    if (frontend.CENTRE_X > x and frontend.CENTRE_X < x + w and frontend.CENTRE_Y > y and frontend.CENTRE_Y < y + h and not manual_control.manual):
                        lock = True
                        lock_size = cv2.getTextSize(
                            "LOCK", FONT, FONT_SCALE, LINE_THICKNESS)[0][0]
                        cv2.rectangle(frame, (frontend.FRAME_WIDTH // 2 - (lock_size // 2), frontend.FRAME_HEIGHT - 38),
                                      (frontend.FRAME_WIDTH // 2 + lock_size - 25, frontend.FRAME_HEIGHT - 20), ui_text_clr, -1)
                        cv2.putText(frame, "LOCK", (frontend.FRAME_WIDTH // 2 - (lock_size // 2),
                                    frontend.FRAME_HEIGHT - 22), FONT, FONT_SCALE, BLACK, LINE_THICKNESS)
                    else:
                        lock = False
                        trk_size = cv2.getTextSize(
                            "TRK", FONT, FONT_SCALE, LINE_THICKNESS)[0][0]
                        cv2.rectangle(frame, (frontend.FRAME_WIDTH // 2 - (trk_size // 2), frontend.FRAME_HEIGHT - 38),
                                      (frontend.FRAME_WIDTH // 2 + trk_size - 20, frontend.FRAME_HEIGHT - 20), ui_text_clr, -1)
                        cv2.putText(frame, "TRK", (frontend.FRAME_WIDTH // 2 - (trk_size // 2),
                                    frontend.FRAME_HEIGHT - 22), FONT, FONT_SCALE, BLACK, LINE_THICKNESS)

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
                    cv2.line(frame, (x + w // 2, y),
                             (x + w // 2, 0), ui_text_clr, 1)
                    # left
                    cv2.line(frame, (x, y + h // 2),
                             (0, y + h // 2), ui_text_clr, 1)
                    # right
                    cv2.line(frame, (x + w, y + h // 2),
                             (frontend.FRAME_WIDTH, y + h // 2), ui_text_clr, 1)
                    # bottom
                    cv2.line(frame, (x + w // 2, y + h),
                             (x + w // 2, frontend.FRAME_HEIGHT), ui_text_clr, 1)

                    if not flt_ctrl_active and not manual_control.manual:
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

    else:
        print("[DRONE] - Low battery")
        drone.end()
except Exception as error:
    print(error)
    drone.end()

print("[DRONE] - CONNECTION TERMINATED")
