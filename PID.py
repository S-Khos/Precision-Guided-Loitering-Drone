import numpy as np
import time


class PID(object):
    def __init__(self, kp, ki, kd, target=0, lower_bound=None, upper_bound=None, init_time=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target = target
        self.error = 0
        self.prev_error = 0
        self.cur_time = init_time if init_time is not None else time.time()
        self.prev_time = self.cur_time - 1
        self.time_diff = 0
        self.proportional = 0
        self.integral = 0
        self.derivative = 0
        self.output = 0
        self.MAX_SPD = upper_bound
        self.MIN_SPD = lower_bound

    def update(self, cur_position, cur_time=None):
        self.error = self.target - cur_position
        self.cur_time = cur_time if cur_time is not None else time.time()
        self.time_diff = self.cur_time - self.prev_time
        self.proportional = self.error
        self.integral += self.error * (self.time_diff)
        self.derivative = (self.error - self.prev_error) / (self.time_diff)
        self.prev_error = self.error
        self.prev_time = cur_time

        self.output = self.kp * self.proportional + \
            self.ki * self.integral + self.kd * self.derivative

        if (self.MAX_SPD is not None):
            if (self.output > self.MAX_SPD):
                self.output = self.MAX_SPD
        if (self.MIN_SPD is not None):
            if self.output < self.MIN_SPD:
                self.output = self.MIN_SPD

        return (self.output, self.cur_time)

    def reset(self):
        self.error = 0
        self.prev_error = 0
        self.integral = 0
        self.derivative = 0
        self.proportional = 0
        self.output = 0
        self.cur_time = 0
        self.prev_time = 0
        self.time_diff = 0
