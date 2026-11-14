"""Bound acceleration and require continuous clearance before resuming motion."""
import math
import numpy as np


class MotionLimiter:
    def __init__(self, linear_acceleration=.15, angular_acceleration=.6, clear_time=.4):
        self.linear_acceleration = linear_acceleration
        self.angular_acceleration = angular_acceleration
        self.clear_time = clear_time
        if not all(math.isfinite(x) and x > 0 for x in
                   (linear_acceleration,angular_acceleration,clear_time)):
            raise ValueError('Motion acceleration and clear time must be finite and positive')
        self.reset()

    def reset(self):
        self.velocity = np.zeros(2)
        self.last_time = self.clear_since = None
        self.status = 'Stopped'

    def update(self, v, w, now):
        target = np.array([v,w],dtype=float)
        if not np.isfinite(target).all() or not np.any(target):
            self.reset()  # Stops, faults and missing inputs bypass smoothing.
            return 0.,0.
        dt = 0. if self.last_time is None else now-self.last_time
        if dt < 0 or dt > .5:
            self.reset()
            dt = 0.
        self.last_time = now
        if self.clear_since is None:
            self.clear_since = now
        if now-self.clear_since < self.clear_time:
            self.status = 'Waiting for continuously clear corridor'
            return 0.,0.
        limit = np.array([self.linear_acceleration,self.angular_acceleration])*min(dt,.2)
        self.velocity += np.clip(target-self.velocity,-limit,limit)
        # Never continue a forward movement while the planner requests a turn,
        # or keep turning after it requests straight travel.
        self.velocity[target == 0] = 0.
        self.status = 'Moving with bounded acceleration'
        return tuple(float(x) for x in self.velocity)
