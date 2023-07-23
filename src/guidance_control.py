import time
import threading
from pid import PID


class GuidanceControl(object):
    def __init__(self, state):
        self.state = state
        self.YAW_PID = [0.30, 0.01, 0.15]  # 0.32, 0, 0.06
        self.Y_PID = [0.2, 0, 0.05]  # 0.1, 0.3, 0.3,
        self.X_PID = [0.25, 0.01, 0.12]
        self.yaw_pivot = 60
        self.max_h_throttle = 0
        self.switch_yaw = False

    def init_guidance_control(self):
        self.state.GS_thread = threading.Thread(
            target=self.update, daemon=True)
        self.state.GS_thread.start()
        self.state.GS_active = True

    def update(self):
        print("[GUIDANCE CONTROL] - ACTIVE")
        try:
            yaw_pid = PID(self.YAW_PID[0], self.YAW_PID[1], self.YAW_PID[2],
                          self.state.CENTRE_X, -100, 100)
            x_pid = PID(self.X_PID[0], self.X_PID[1], self.X_PID[2],
                        self.state.CENTRE_X, -100, 100)
            y_pid = PID(self.Y_PID[0], self.Y_PID[1], self.Y_PID[2],
                        self.state.CENTRE_Y - 200, -100, self.max_h_throttle)
            while self.state.TR_active and not self.state.KC_manual and self.state.TR_return:
                x, y, w, h = int(self.state.TR_bbox[0]), int(self.state.TR_bbox[1]), int(
                    self.state.TR_bbox[2]), int(self.state.TR_bbox[3])
                targetX = int(x + w / 2)
                targetY = int(y + h / 2)

                self.max_h_throttle = 100 if self.state.GS_lock else 0
                self.state.yaw_Throttle, yaw_time = yaw_pid.update(targetX)
                self.state.lr_Throttle, x_time = x_pid.update(targetX)
                self.state.h_Throttle, y_time = y_pid.update(targetY)

                self.state.fb_Throttle = 40 if self.state.h_Throttle < -80 else 90

                self.switch_yaw = True if abs(
                    -self.state.lr_Throttle) >= self.yaw_pivot else False

                if self.state.drone.send_rc_control:
                    self.state.drone.send_rc_control(-self.state.lr_Throttle if self.switch_yaw else 0, self.state.fb_Throttle if self.state.altitude >
                                                     1.3 and self.state.GS_dive else 0, self.state.h_Throttle if self.state.GS_dive else 0, -self.state.yaw_Throttle if not self.switch_yaw else 0)
                time.sleep(0.0145)

        except Exception as error:
            print("[GUIDANCE CONTROL] - ", error)

        self.state.GS_active = False
        self.state.KC_manual = True
        self.state.GS_dive = False
        self.state.drone.send_rc_control(0, 0, 0, 0)
        self.state.reset_throttle()
        print("[GUIDANCE CONTROL] - TERMINATED")
