import time
import threading
from pid import PID


class GuidanceControl(object):
    def __init__(self, state):
        self.state = state
        self.YAW_PID = [0.35, 0.02, 0.14]  # 0.32, 0, 0.06
        self.Y_PID = [1.4, 0.1, 0.8]  # 0.1, 0.3, 0.3,
        self.X_PID = [0.7, 0.0, 0.35]

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
                        self.state.CENTRE_X, -80, 80)
            y_pid = PID(self.Y_PID[0], self.Y_PID[1], self.Y_PID[2],
                        self.state.CENTRE_Y, -100, 100)
            while self.state.TR_active and not self.state.KC_manual and self.state.TR_return:
                x, y, w, h = int(self.state.TR_bbox[0]), int(self.state.TR_bbox[1]), int(
                    self.state.TR_bbox[2]), int(self.state.TR_bbox[3])
                targetX = int(x + w / 2)
                targetY = int(y + h / 2)
                yaw_velocity, yaw_time = yaw_pid.update(targetX)
                x_velocity, x_time = x_pid.update(targetX)
                y_velocity, y_time = y_pid.update(targetY)
                if self.state.drone.send_rc_control:
                    self.state.drone.send_rc_control(
                        -x_velocity if abs(x_velocity) > 65 else 0, 90 if self.state.altitude > 1.5 and self.state.GS_dive else 0, y_velocity if dive else 0, -yaw_velocity if abs(yaw_velocity) < 65 else 0)
                time.sleep(0.014)
            self.state.GS_active = False
            self.state.KC_manual = True
            self.state.drone.send_rc_control(0, 0, 0, 0)
            print("[GUIDANCE CONTROL] - TERMINATED")

        except Exception as error:
            self.state.GS_active = False
            self.state.KC_manual = True
            self.state.drone.send_rc_control(0, 0, 0, 0)
            print("[GUIDANCE CONTROL] - ", error)
