class State(object):
    def __init__(self, drone):
        self.drone = drone

        self.FRAME_WIDTH = 960
        self.FRAME_HEIGHT = 720
        self.CENTRE_X = int(FRAME_WIDTH / 2)
        self.CENTRE_Y = int(FRAME_HEIGHT / 2)
        self.FONT = cv2.FONT_HERSHEY_COMPLEX
        self.FONT_SCALE = .6
        self.RED = (0, 0, 255)
        self.GREEN = (0, 255, 0)
        self.BLUE = (255, 0, 0)
        self.WHITE = (255, 255, 255)
        self.BLACK = (0, 0, 0)
        self.UI_COLOUR = WHITE
        self.LINE_THICKNESS = 1

        self.KC_manual = True
        self.KC_default_dist = 30
        self.KC_designator_delta = 30
        self.KC_designator_roi_size = [100, 100]

        self.TR_active = False
        self.TR_reset = True
        self.TR_tracker = None
        self.TR_thread = None
        self.TR_designator_frame = None
        self.TR_bbox = None
        self.TR_thread_lock = threading.Lock()

        self.GS_active = False
        self.GS_thread = None
        self.GS_lock = False
        self.GS_dive = False

        self.CC_cursor_pos = [self.CENTRE_X, self.CENTRE_Y]
