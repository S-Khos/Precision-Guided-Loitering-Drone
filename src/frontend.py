import cv2
import numpy as np
import time
import threading
import math


class FrontEnd(object):

    FRAME_WIDTH = 960
    FRAME_HEIGHT = 720
    CENTRE_X = int(FRAME_WIDTH / 2)
    CENTRE_Y = int(FRAME_HEIGHT / 2)
    FONT = cv2.FONT_HERSHEY_COMPLEX
    FONT_SCALE = .6
    RED = (0, 0, 255)
    GREEN = (0, 255, 0)
    BLUE = (255, 0, 0)
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    UI_COLOUR = WHITE
    LINE_THICKNESS = 1

    def __init__(self, backend, manual_control, cursor_control, tracker, guidance_system):
        self.backend = backend
        self.manual_control = keyboard_control
        self.tracker = tracker
        self.cursor_control = cursor_control
        self.guidance_system = guidance_system
        self.frame = None
        self.designator_frame = None

    def update(self, frame):
        self.frame = frame
        self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
        self.designator_frame = self.frame.copy()

        # fps
        fps = self.backend.get_fps()
        fps_size = cv2.getTextSize("FPS  {}".format(
            fps), frontend.FONT, frontend.FONT_SCALE, frontend.LINE_THICKNESS)[0][0]
        cv2.putText(self.frame, "FPS  {}".format(fps), (frontend.FRAME_WIDTH -
                    fps_size - 5, 25), frontend.FONT, frontend.FONT_SCALE, frontend.UI_COLOUR, frontend.LINE_THICKNESS)

        # top left
        cv2.putText(self.frame, "PWR   {}%".format(self.backend.get_battery()),
                    (5, 25), frontend.FONT, frontend.FONT_SCALE, frontend.UI_COLOUR, frontend.LINE_THICKNESS)
        cv2.putText(self.frame, "TMP  {} C".format(self.backend.get_barometer()),
                    (5, 55), frontend.FONT, frontend.FONT_SCALE, frontend.UI_COLOUR, frontend.LINE_THICKNESS)

        # crosshair
        cv2.line(self.frame, (frontend.CENTRE_X - 20, frontend.CENTRE_Y),
                 (frontend.CENTRE_X - 10, frontend.CENTRE_Y), frontend.UI_COLOUR, 2)
        cv2.line(self.frame, (frontend.CENTRE_X + 20, frontend.CENTRE_Y),
                 (frontend.CENTRE_X + 10, frontend.CENTRE_Y), frontend.UI_COLOUR, 2)
        cv2.line(self.frame, (frontend.CENTRE_X, frontend.CENTRE_Y - 20),
                 (frontend.CENTRE_X, frontend.CENTRE_Y - 10), frontend.UI_COLOUR, 2)
        cv2.line(self.frame, (frontend.CENTRE_X, frontend.CENTRE_Y + 20),
                 (frontend.CENTRE_X, frontend.CENTRE_Y + 10), frontend.UI_COLOUR, 2)

        # crosshair stats
        spd_size = cv2.getTextSize(
            "SPD  {} CM/S".format(self.backend.get_speed_mag()), frontend.FONT, frontend.FONT_SCALE, frontend.LINE_THICKNESS)[0][0]
        cv2.putText(self.frame, "SPD  {} CM/S".format(self.backend.get_speed_mag()), ((frontend.CENTRE_X) - 90 -
                    spd_size, (frontend.CENTRE_Y) - 100), frontend.FONT, frontend.FONT_SCALE, frontend.UI_COLOUR, frontend.LINE_THICKNESS)
        cv2.putText(self.frame, "ALT  {:.1f} FT".format(self.backend.get_altitude()), ((
            frontend.CENTRE_X) + 90, (frontend.CENTRE_Y) - 100), frontend.FONT, frontend.FONT_SCALE, frontend.UI_COLOUR, frontend.LINE_THICKNESS)

        # bottom left telemtry
        cv2.putText(self.frame, "BRM  {}".format(self.backend.get_barometer()),
                    (5, frontend.FRAME_HEIGHT - 100), frontend.FONT, frontend.FONT_SCALE, frontend.UI_COLOUR, frontend.LINE_THICKNESS)
        cv2.putText(self.frame, "SPD  {}  {}  {}".format(self.backend.get_speed_x(), self.backend.get_speed_y(
        ), self.backend.get_speed_z()), (5, frontend.FRAME_HEIGHT - 70), frontend.FONT, frontend.FONT_SCALE, frontend.UI_COLOUR, frontend.LINE_THICKNESS)
        cv2.putText(self.frame, "ACC  {}  {}  {}".format(self.backend.get_acceleration_x(), self.backend.get_acceleration_y(
        ), self.backend.get_acceleration_z()), (5, frontend.FRAME_HEIGHT - 40), frontend.FONT, frontend.FONT_SCALE, frontend.UI_COLOUR, frontend.LINE_THICKNESS)
        cv2.putText(self.frame, "YPR  {}  {}  {}".format(self.backend.get_yaw(), self.backend.get_pitch(
        ), self.backend.get_roll()), (5, frontend.FRAME_HEIGHT - 10), frontend.FONT, frontend.FONT_SCALE, frontend.UI_COLOUR, frontend.LINE_THICKNESS)

        time_size = cv2.getTextSize(
            "T + {}".format(self.backend.get_flight_time()), frontend.FONT, frontend.FONT_SCALE, frontend.LINE_THICKNESS)[0][0]
        cv2.putText(self.frame, "T + {}".format(self.backend.get_flight_time()),
                    (frontend.FRAME_WIDTH - time_size - 5, 55), frontend.FONT, frontend.FONT_SCALE, frontend.UI_COLOUR, frontend.LINE_THICKNESS)

        # bottom compass
        cv2.circle(self.frame, (frontend.FRAME_WIDTH - 60, frontend.FRAME_HEIGHT - 60),
                   50, UI_COLOUR, 1)
        cv2.arrowedLine(self.frame, (frontend.FRAME_WIDTH - 60, frontend.FRAME_HEIGHT - 60), (int(-50 * math.cos(math.radians(self.backend.get_yaw() + 90)) +
                        frontend.FRAME_WIDTH - 60), int((frontend.FRAME_HEIGHT - 60) - (50 * math.sin(math.radians(self.backend.get_yaw() + 90))))), frontend.UI_COLOUR, 1, tipLength=.15)

        # top center
        if (self.manual_control.manual and not self.guidance_system.active):
            cv2.rectangle(self.frame, (frontend.CENTRE_X - 20, 10),
                          (frontend.CENTRE_X + 29, 28), frontend.UI_COLOUR, -1)
            cv2.putText(self.frame, "CTRL", (frontend.CENTRE_X - 20, 25),
                        frontend.FONT, frontend.FONT_SCALE, frontend.BLACK, frontend.LINE_THICKNESS)
        else:
            cv2.rectangle(self.frame, (frontend.CENTRE_X - 20, 10),
                          (frontend.CENTRE_X + 31, 28), frontend.UI_COLOUR, -1)
            cv2.putText(self.frame, "AUTO", (frontend.CENTRE_X - 20, 25),
                        frontend.FONT, frontend.FONT_SCALE, frontend.BLACK, frontend.LINE_THICKNESS)

        # designator cursor
        if not self.tracker.active:
            cv2.rectangle(self.designator_frame, (cursor_control.cursor_pos[0], cursor_control.cursor_pos[1]), (
                cursor_control.cursor_pos[0] + manual_control.designator_roi_size[0], cursor_control.cursor_pos[1] + manual_control.designator_roi_size[1]), frontend.UI_COLOUR, 1)
            # top
            cv2.line(self.designator_frame, (cursor_control.cursor_pos[0] + manual_control.designator_roi_size[0] // 2, cursor_control.cursor_pos[1]),
                     (cursor_control.cursor_pos[0] + manual_control.designator_roi_size[0] // 2, 0), frontend.UI_COLOUR, 1)
            # left
            cv2.line(self.designator_frame, (cursor_control.cursor_pos[0], cursor_control.cursor_pos[1] + manual_control.designator_roi_size[1] // 2),
                     (0, cursor_control.cursor_pos[1] + manual_control.designator_roi_size[1] // 2), frontend.UI_COLOUR, 1)
            # right
            cv2.line(self.designator_frame, (cursor_control.cursor_pos[0] + manual_control.designator_roi_size[0], cursor_control.cursor_pos[1] + manual_control.designator_roi_size[1] // 2),
                     (frontend.FRAME_WIDTH, cursor_control.cursor_pos[1] + manual_control.designator_roi_size[1] // 2), frontend.UI_COLOUR, 1)
            # bottom
            cv2.line(self.designator_frame, (cursor_control.cursor_pos[0] + manual_control.designator_roi_size[0] // 2, cursor_control.cursor_pos[1] + manual_control.designator_roi_size[1]),
                     (cursor_control.cursor_pos[0] + manual_control.designator_roi_size[0] // 2, frontend.FRAME_HEIGHT), frontend.UI_COLOUR, 1)

        # active tracking / lock
        if tracker.active:
            x, y, w, h = tracker.get_bbox()
            if (frontend.CENTRE_X > x and frontend.CENTRE_X < x + w and frontend.CENTRE_Y > y and frontend.CENTRE_Y < y + h and not manual_control.manual):
                self.guidance_system.lock = True
                lock_size = cv2.getTextSize(
                    "LOCK", frontend.FONT, frontend.FONT_SCALE, frontend.LINE_THICKNESS)[0][0]
                cv2.rectangle(self.frame, (frontend.CENTRE_X - (lock_size // 2), frontend.FRAME_HEIGHT - 38),
                              (frontend.CENTRE_X + lock_size - 25, frontend.FRAME_HEIGHT - 20), frontend.UI_COLOUR, -1)
                cv2.putText(self.frame, "LOCK", (frontend.CENTRE_X - (lock_size // 2),
                            frontend.FRAME_HEIGHT - 22), frontend.FONT, frontend.FONT_SCALE, frontend.BLACK, frontend.LINE_THICKNESS)
            else:
                self.guidance_system.lock = False
                trk_size = cv2.getTextSize(
                    "TRK", frontend.FONT, frontend.FONT_SCALE, frontend.LINE_THICKNESS)[0][0]
                cv2.rectangle(self.frame, (frontend.CENTRE_X - (trk_size // 2), frontend.FRAME_HEIGHT - 38),
                              (frontend.CENTRE_X + trk_size - 20, frontend.FRAME_HEIGHT - 20), frontend.UI_COLOUR, -1)
                cv2.putText(self.frame, "TRK", (frontend.CENTRE_X - (trk_size // 2),
                            frontend.FRAME_HEIGHT - 22), frontend.FONT, frontend.FONT_SCALE, frontend.BLACK, frontend.LINE_THICKNESS)

            cv2.line(self.frame, (x, y), (x + 20, y),
                     frontend.RED if manual_control.dive else frontend.UI_COLOUR, 2)
            cv2.line(self.frame, (x, y), (x, y + 20),
                     frontend.RED if manual_control.dive else frontend.UI_COLOUR, 2)
            cv2.line(self.frame, (x, y + h), (x, y + h - 20),
                     frontend.RED if manual_control.dive else frontend.UI_COLOUR, 2)
            cv2.line(self.frame, (x, y + h), (x + 20, y + h),
                     frontend.RED if manual_control.dive else frontend.UI_COLOUR, 2)

            cv2.line(self.frame, (x + w, y), (x + w - 20, y),
                     frontend.RED if manual_control.dive else frontend.UI_COLOUR, 2)
            cv2.line(self.frame, (x + w, y), (x + w, y + 20),
                     frontend.RED if manual_control.dive else frontend.UI_COLOUR, 2)
            cv2.line(self.frame, (x + w, y + h), (x + w, y + h - 20),
                     frontend.RED if manual_control.dive else frontend.UI_COLOUR, 2)
            cv2.line(self.frame, (x + w, y + h), (x + w - 20, y + h),
                     frontend.RED if manual_control.dive else frontend.UI_COLOUR, 2)

            cv2.circle(self.frame, (x + w // 2, y + h // 2), 3,
                       frontend.RED if manual_control.dive else frontend.UI_COLOUR, -1)
            # top
            cv2.line(self.frame, (x + w // 2, y),
                     (x + w // 2, 0), frontend.UI_COLOUR, 1)
            # left
            cv2.line(self.frame, (x, y + h // 2),
                     (0, y + h // 2), frontend.UI_COLOUR, 1)
            # right
            cv2.line(self.frame, (x + w, y + h // 2),
                     (frontend.FRAME_WIDTH, y + h // 2), frontend.UI_COLOUR, 1)
            # bottom
            cv2.line(self.frame, (x + w // 2, y + h),
                     (x + w // 2, frontend.FRAME_HEIGHT), frontend.UI_COLOUR, 1)

        return (self.frame, self.designator_frame)

    def get_frame_size(self):
        return (FrontEnd.FRAME_WIDTH, FrontEnd.FRAME_HEIGHT)

    def get_frame_centre(self):
        return (FrontEnd.CENTRE_X, FrontEnd.CENTRE_Y)

    def get_designator_frame(self):
        return self.designator_frame

    def get_frame(self):
        return self.frame
