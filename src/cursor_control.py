import cv2

# ----- fix if statement, move tracker init to backend update instead and remove the need for tracker


class CursorControl(object):
    def __init__(self, state, tracker):
        self.state = state
        self.tracker = tracker

    def event_handler(self, event, x, y, flags, param):
        self.state.CC_cursor_pos[0] = int(
            x - self.state.KC_designator_roi_size[0] / 2)
        self.state.CC_cursor_pos[1] = int(
            y - self.state.KC_designator_roi_size[1] / 2)
        if event == cv2.EVENT_LBUTTONDOWN:
            if not self.state.TR_active and self.state.TR_reset:
                self.tracker.init_tracker()
            else:
                self.state.TR_reset = True
                self.state.TR_active = False
