import cv2


class CursorControl(object):
    def __init__(self, backend):
        self.backend = backend

    def event_handler(self, event, x, y, flags, param):
        self.backend.CC_cursor_pos[0] = int(
            x - self.backend.KC_designator_roi_size[0] / 2)
        self.backend.CC_cursor_pos[1] = int(
            y - self.backend.KC_designator_roi_size[1] / 2)
        if event == cv2.EVENT_LBUTTONDOWN:
            if not self.backend.TR_active and self.backend.TR_reset:
                self.backend.TR_active = True
            else:
                self.backend.TR_reset = True
                self.backend.TR_active = False
