"""Planar actuator response, with optional sampled pure input delay.

Time and delay use simulation seconds. No ROS or MuJoCo dependency: the same
plant can be exercised in deterministic offline closed-loop checks.
"""
from collections import deque
import math


class BaseResponse:
    def __init__(self, enabled=False, linear_tau=0.35, angular_tau=0.60,
                 linear_gain=1.0, angular_gain=1.0,
                 linear_delay=0.0, angular_delay=0.0):
        self.enabled = bool(enabled)
        self.tau = (float(linear_tau), float(angular_tau))
        self.gain = (float(linear_gain), float(angular_gain))
        self.delay = (float(linear_delay), float(angular_delay))
        if any(not math.isfinite(v) or v <= 0 for v in (*self.tau, *self.gain)):
            raise ValueError("base response gains/time constants must be finite and positive")
        if any(not math.isfinite(v) or v < 0 for v in self.delay):
            raise ValueError("base response delays must be finite and non-negative")
        self.reset()

    def reset(self):
        self.velocity = [0.0, 0.0]
        self.history = [deque(), deque()]
        self.delayed = [0.0, 0.0]
        self.last_time = None

    def step(self, now, dt, linear_command, angular_command):
        if not all(math.isfinite(v) for v in (now, dt, linear_command, angular_command)) or dt <= 0:
            raise ValueError("base response step requires finite values and positive dt")
        if self.last_time is not None and now < self.last_time:
            raise ValueError("base response time must be monotonic")
        self.last_time = now
        commands = (linear_command, angular_command)
        if not self.enabled:
            self.velocity[:] = commands
            return tuple(self.velocity)
        for axis in range(2):
            queue = self.history[axis]
            queue.append((now, commands[axis]))
            cutoff = now - self.delay[axis]
            while queue and queue[0][0] <= cutoff + 1e-12:
                self.delayed[axis] = queue.popleft()[1]
            alpha = -math.expm1(-dt / self.tau[axis])
            self.velocity[axis] += alpha * (
                self.gain[axis] * self.delayed[axis] - self.velocity[axis])
        return tuple(self.velocity)
