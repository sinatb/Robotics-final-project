import time
from typing import Any
class PID(object):
    def __init__(self,kp,ki,kd,goal):
        self.kp, self.ki, self.kd = kp, ki, kd
        self._p, self._i, self._d = None, None, None
        self.goal = goal
        self.time = time.monotonic
        
        self.time_step = 0.01

        self._prev_time = None
        self._prev_output = None
        self._prev_error = None
        self._prev_input = None

    def __call__(self, input_):
        
        now = self.time()
        dt = now - self._prev_time if (now - self._prev_time) else 1e-16

        if self._prev_output is not None and dt < self.time_step:
            return self._prev_output

        error = self.goal - input_
        d_input = input_ - (self._prev_input if (self._prev_input is not None) else input_)
        d_error = error - (self._prev_error if (self._prev_error is not None) else error)
        # Proportional Term
        self._p = self.kp * error
        # Integral Term
        self._i += self.ki * error * dt
        # Derivative Term
        self._d = -self.kd * d_input / dt

        output = self._p + self._i + self._d

        self._prev_error = error
        self._prev_input = input_
        self._prev_output = output
        self._prev_time = now

        return output