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

    def __init__(self, backend):
        self.backend = backend
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
        cv2.putText(frame, "FPS  {}".format(fps), (frontend.FRAME_WIDTH -
                    fps_size - 5, 25), frontend.FONT, frontend.FONT_SCALE, frontend.UI_COLOUR, frontend.LINE_THICKNESS)

        # top left
        cv2.putText(frame, "PWR   {}%".format(backend.get_battery()),
                    (5, 25), frontend.FONT, frontend.FONT_SCALE, frontend.UI_COLOUR, frontend.LINE_THICKNESS)
        cv2.putText(frame, "TMP  {} C".format(backend.get_barometer()),
                    (5, 55), frontend.FONT, frontend.FONT_SCALE, frontend.UI_COLOUR, frontend.LINE_THICKNESS)

        # crosshair
        cv2.line(frame, (int(frontend.FRAME_WIDTH / 2) - 20, int(frontend.FRAME_HEIGHT / 2)),
                 (int(frontend.FRAME_WIDTH / 2) - 10, int(frontend.FRAME_HEIGHT / 2)), frontend.UI_COLOUR, 2)
        cv2.line(frame, (int(frontend.FRAME_WIDTH / 2) + 20, int(frontend.FRAME_HEIGHT / 2)),
                 (int(frontend.FRAME_WIDTH / 2) + 10, int(frontend.FRAME_HEIGHT / 2)), frontend.UI_COLOUR, 2)
        cv2.line(frame, (int(frontend.FRAME_WIDTH / 2), int(frontend.FRAME_HEIGHT / 2) - 20),
                 (int(frontend.FRAME_WIDTH / 2), int(frontend.FRAME_HEIGHT / 2) - 10), frontend.UI_COLOUR, 2)
        cv2.line(frame, (int(frontend.FRAME_WIDTH / 2), int(frontend.FRAME_HEIGHT / 2) + 20),
                 (int(frontend.FRAME_WIDTH / 2), int(frontend.FRAME_HEIGHT / 2) + 10), frontend.UI_COLOUR, 2)

        # crosshair stats
        spd_size = cv2.getTextSize(
            "SPD  {} CM/S".format(backend.get_speed_mag()), frontend.FONT, frontend.FONT_SCALE, frontend.LINE_THICKNESS)[0][0]
        cv2.putText(frame, "SPD  {} CM/S".format(backend.get_speed_mag()), ((frontend.FRAME_WIDTH // 2) - 90 -
                    spd_size, (frontend.FRAME_HEIGHT // 2) - 100), frontend.FONT, frontend.FONT_SCALE, frontend.frontend.UI_COLOUR, frontend.LINE_THICKNESS)
        cv2.putText(frame, "ALT  {:.1f} FT".format(backend.get_altitude()), ((
            frontend.FRAME_WIDTH // 2) + 90, (frontend.FRAME_HEIGHT // 2) - 100), frontend.FONT, frontend.FONT_SCALE, frontend.UI_COLOUR, frontend.LINE_THICKNESS)

        # bottom left telemtry
        cv2.putText(frame, "BRM  {}".format(backend.get_barometer()),
                    (5, frontend.FRAME_HEIGHT - 100), FONT, FONT_SCALE, UI_COLOUR, LINE_THICKNESS)
        cv2.putText(frame, "SPD  {}  {}  {}".format(backend.get_speed_x(), backend.get_speed_y(
        ), backend.get_speed_z()), (5, frontend.FRAME_HEIGHT - 70), FONT, FONT_SCALE, UI_COLOUR, LINE_THICKNESS)
        cv2.putText(frame, "ACC  {}  {}  {}".format(backend.get_acceleration_x(), backend.get_acceleration_y(
        ), backend.get_acceleration_z()), (5, frontend.FRAME_HEIGHT - 40), FONT, FONT_SCALE, UI_COLOUR, LINE_THICKNESS)
        cv2.putText(frame, "YPR  {}  {}  {}".format(backend.get_yaw(), backend.get_pitch(
        ), backend.get_roll()), (5, frontend.FRAME_HEIGHT - 10), FONT, FONT_SCALE, UI_COLOUR, LINE_THICKNESS)

        time_size = cv2.getTextSize(
            "T + {}".format(backend.get_flight_time()), FONT, FONT_SCALE, LINE_THICKNESS)[0][0]
        cv2.putText(frame, "T + {}".format(drone.get_flight_time()),
                    (frontend.FRAME_WIDTH - time_size - 5, 55), FONT, FONT_SCALE, UI_COLOUR, LINE_THICKNESS)

        # bottom compass
        cv2.circle(frame, (frontend.FRAME_WIDTH - 60, frontend.FRAME_HEIGHT - 60),
                   50, ui_text_clr, 1)
        cv2.arrowedLine(frame, (frontend.FRAME_WIDTH - 60, frontend.FRAME_HEIGHT - 60), (int(-50 * math.cos(math.radians(drone.get_yaw() + 90)) +
                        frontend.FRAME_WIDTH - 60), int((frontend.FRAME_HEIGHT - 60) - (50 * math.sin(math.radians(drone.get_yaw() + 90))))), ui_text_clr, 1, tipLength=.15)

        return (self.frame, self.designator_frame)

    def get_frame_size(self):
        return (FrontEnd.FRAME_WIDTH, FrontEnd.FRAME_HEIGHT)

    def get_frame_centre(self):
        return (FrontEnd.CENTRE_X, FrontEnd.CENTRE_Y)

    def get_designator_frame(self):
        return self.designator_frame

    def get_frame(self):
        return self.frame
