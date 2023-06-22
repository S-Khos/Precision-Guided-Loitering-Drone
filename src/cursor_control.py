import cv2


class CursorControl(object):
    def __init__(self, state):
        self.state = state

    def event_handler(self, event, x, y, flags, param):
        self.state.CC_cursor_pos[0] = int(
            x - self.state.KC_designator_roi_size[0] / 2)
        self.state.CC_cursor_pos[1] = int(
            y - self.state.KC_designator_roi_size[1] / 2)
        if event == cv2.EVENT_LBUTTONDOWN:
            self.state.TR_active = not self.state.TR_active
