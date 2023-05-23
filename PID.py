import numpy as np
import time

class PID(object):
    def __init__(self, kp, ki, kd, target, lower_bound=None, upper_bound=None, use_time=True):
        self.use_time = use_time
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target = target
        self.error = 0
        self.prev_error = 0
        self.prev_time = time.time() if self.use_time == True else None
        self.proportional = 0
        self.integral = 0
        self.derivative = 0
        self.output = 0
        self.MAX_SPD = upper_bound
        self.MIN_SPD = lower_bound
    
    def compute(self, cur_position):
        self.error = self.target - cur_position

        if (self.use_time):
            cur_time = time.time()
            self.proportional = self.error
            self.integral += self.error * (cur_time - self.prev_time)
            self.derivative = (self.error - self.prev_error) / (cur_time - self.prev_time)
            self.prev_error = self.error
            self.prev_time = cur_time
        else:
            self.proportional = self.error
            self.integral = self.error # changed from += to =
            self.derivative = self.error - self.prev_error
            self.prev_error = self.error
        
        self.output = int(self.kp * self.proportional + self.ki * self.integral + self.kd * self.derivative)

        if (self.MAX_SPD is not None):
            if (self.output > self.MAX_SPD):
                self.output = self.MAX_SPD
        if (self.MIN_SPD is not None):
            if self.output < self.MIN_SPD:
                self.output = self.MIN_SPD
        
        return self.output

    def reset(self):
        self.error = 0
        self.prev_error = 0
        self.time = None
        self.integral = 0
        self.derivative = 0
        self.proportional = 0
        self.output = 0
        
