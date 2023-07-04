import cv2
import threading


class State(object):
    def __init__(self, drone):
        self.drone = drone
        self.frame = None
        self.designator_frame = None
        self.FRAME_WIDTH = 960
        self.FRAME_HEIGHT = 720
        self.CENTRE_X = int(self.FRAME_WIDTH / 2)
        self.CENTRE_Y = int(self.FRAME_HEIGHT / 2)
        self.FONT = cv2.FONT_HERSHEY_COMPLEX
        self.FONT_SCALE = .6
        self.RED = (0, 0, 255)
        self.GREEN = (0, 255, 0)
        self.BLUE = (255, 0, 0)
        self.WHITE = (255, 255, 255)
        self.BLACK = (0, 0, 0)
        self.UI_COLOUR = self.GREEN
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
        self.TR_bbox = []
        self.TR_return = False
        self.TR_thread_lock = threading.Lock()

        self.GS_active = False
        self.GS_thread = None
        self.GS_lock = False
        self.GS_dive = False
        self.forward_Throttle = 0
        self.yaw_Throttle = 0
        self.lr_Throttle = 0
        self.y_Throttle = 0

        self.CC_cursor_pos = [self.CENTRE_X - self.KC_designator_roi_size[0] //
                              2, self.CENTRE_Y - self.KC_designator_roi_size[1] // 2]

        self.altitude = 0
        self.battery = 0
        self.temperature = 0
        self.flight_time = 0
        self.barometer = 0
        self.acceleration_x = 0
        self.acceleration_y = 0
        self.acceleration_z = 0
        self.x_speed = 0
        self.y_speed = 0
        self.z_speed = 0
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        self.state_dict = {}

    def update(self):
        self.state_dict = self.drone.get_current_state()
        self.altitude = (self.state_dict['tof'] / 30.48)
        self.battery = int(self.state_dict['bat'])
        self.temperature = int(self.drone.get_temperature())
        self.flight_time = int(self.state_dict['time'])
        self.barometer = int(self.state_dict['baro'] * 3.281)
        self.acceleration_x = int(self.state_dict['agx'])
        self.acceleration_y = int(self.state_dict['agy'])
        self.acceleration_z = int(self.state_dict['agz'])
        self.x_speed = abs(int(self.state_dict['vgx']))
        self.y_speed = abs(int(self.state_dict['vgy']))
        self.z_speed = abs(int(self.state_dict['vgz']))
        self.yaw = int(self.state_dict['yaw'])
        self.pitch = int(self.state_dict['pitch'])
        self.roll = int(self.state_dict['roll'])
