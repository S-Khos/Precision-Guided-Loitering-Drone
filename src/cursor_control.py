from backend import BackEnd
from tracker import Tracker
import cv2


class CursorControl(BackEnd, Tracker):
    def __init__(self):
        super().__init__()

    def event_handler(self, event, x, y, flags, param):
        self.CC_cursor_pos[0] = int(
            x - self.KC_designator_roi_size[0] / 2)
        self.CC_cursor_pos[1] = int(
            y - self.KC_designator_roi_size[1] / 2)
        if event == cv2.EVENT_LBUTTONDOWN:
            # ------------- put this in backend or tracker, check for tr reset and tr active -----------------
            if not self.TR_active and self.TR_reset:
                self.init_tracker()
            else:
                self.TR_reset = True
                self.TR_active = False
