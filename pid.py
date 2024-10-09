#!/usr/bin/env python3
import math
import numpy as np

class PID:
    def __init__(self, min_val, max_val, kp, ki, kd):
        self.min_val = min_val
        self.max_val = max_val
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0

    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value
        self.integral += error
        self.derivative = error - self.prev_error
        if setpoint == 0 and self.prev_error == 0:
            self.integreal = 0
        pid = self.kp * error + self.ki*self.integral + self.kd * self.derivative
        self.prev_error = error
        return np.clip(pid, self.min_val, self.max_val)
