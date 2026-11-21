"""Real-time localization while driving: odometry + marker EKF (no ROS imports).

State (x, y, h): robot position in the field frame (metres) and heading (rad).

  predict  (every odometry step, robot-frame motion dx, dy, dh):
      x += cos(h) dx - sin(h) dy
      y += sin(h) dx + cos(h) dy
      h += dh
      P  = F P F^T + Q            Q grows with distance driven and angle turned

  update   (every marker seen, ANY number, one at a time):
      predicted range  d = |marker - robot|
      predicted bearing a = atan2(dy, dx) - h
      H rows (same J as triangulation_advanced.py):
          range   [-dx/d,    -dy/d,     0]
          bearing [ dy/d^2,  -dx/d^2,  -1]
      S = H P H^T + R,   K = P H^T S^-1
      state += K (z - prediction),   P = (I - K H) P

One marker is enough for an update: it corrects the direction along the
range and across the bearing; odometry carries the rest. Two or more markers
over time fix all three states.

Outliers (wrong ID, reflections, a bad pose) are rejected with a Mahalanobis
gate: m^2 = innovation^T S^-1 innovation must stay below a chi-square limit.

Start: from a two-or-more-marker fix (triangulation_advanced), or from a
given start pose with a large uncertainty.
"""

from dataclasses import dataclass
import math

import numpy as np

from .triangulation import Field, wrap
from .triangulation_advanced import NoiseModel, localize_advanced

CHI2_2DOF_99_9 = 13.82         # 2 measurements (range, bearing), 99.9 %


@dataclass
class MotionNoise:
    """Odometry error: standard deviation per metre driven / per radian turned."""

    xy_per_m: float = 0.05          # 5 cm sideways/along per metre driven
    h_per_rad: float = 0.05         # 5 % of every turn
    h_per_m: float = 0.02           # heading drift per metre (wheel slip), rad
    xy_floor: float = 0.002         # small noise even when standing still, per step
    h_floor: float = 0.001


def relative_motion(previous, current):
    """Robot-frame motion (dx, dy, dh) between two odometry poses (x, y, yaw)."""
    ddx, ddy = current[0] - previous[0], current[1] - previous[1]
    c, s = math.cos(previous[2]), math.sin(previous[2])
    return c * ddx + s * ddy, -s * ddx + c * ddy, wrap(current[2] - previous[2])


class MarkerEKF:
    def __init__(self, noise=NoiseModel(), motion=MotionNoise(), gate=CHI2_2DOF_99_9,
                 field_model=Field()):
        self.noise, self.motion, self.gate, self.field = noise, motion, gate, field_model
        self.state = None               # np.array([x, y, h]) once initialised
        self.P = None
        self.accepted = 0
        self.rejected_in_row = 0
        self.last_rejection = ''

    @property
    def initialised(self):
        return self.state is not None

    # ------------------------------------------------------------ start
    def initialise(self, pose, sigma_xy=0.5, sigma_h_deg=20.0):
        self.state = np.array(pose, float)
        self.state[2] = wrap(self.state[2])
        self.P = np.diag([sigma_xy ** 2, sigma_xy ** 2, math.radians(sigma_h_deg) ** 2])
        self.rejected_in_row = 0

    def initialise_from_markers(self, landmarks, inflate=2.0):
        """Absolute fix from 2+ markers in one image. Returns the fix, or None."""
        if sum(m.bearing is not None for m in landmarks) < 2:
            return None
        try:
            fix = localize_advanced(landmarks, self.noise, self.field)
        except (ValueError, np.linalg.LinAlgError):
            return None
        if not self.field.contains((fix.x, fix.y)):
            return None
        self.state = np.array([fix.x, fix.y, fix.h])
        self.P = fix.covariance * inflate ** 2 + np.diag([1e-4, 1e-4, 1e-4])
        self.rejected_in_row = 0
        return fix

    # ------------------------------------------------------------ predict
    def predict(self, dx, dy, dh):
        if not self.initialised:
            return
        x, y, h = self.state
        c, s = math.cos(h), math.sin(h)
        self.state = np.array([x + c * dx - s * dy, y + s * dx + c * dy, wrap(h + dh)])
        F = np.array([[1.0, 0.0, -s * dx - c * dy],
                      [0.0, 1.0, c * dx - s * dy],
                      [0.0, 0.0, 1.0]])
        distance = math.hypot(dx, dy)
        m = self.motion
        sigma_xy = m.xy_per_m * distance + m.xy_floor
        sigma_h = m.h_per_rad * abs(dh) + m.h_per_m * distance + m.h_floor
        Q = np.diag([sigma_xy ** 2, sigma_xy ** 2, sigma_h ** 2])
        self.P = F @ self.P @ F.T + Q

    # ------------------------------------------------------------ update
    def measurement(self, marker, state=None):
        """Predicted (range, bearing) and H for one marker."""
        x, y, h = self.state if state is None else state
        dx, dy = marker.position[0] - x, marker.position[1] - y
        d = max(math.hypot(dx, dy), 1e-6)
        predicted = np.array([d, math.atan2(dy, dx) - h])
        H = np.array([[-dx / d, -dy / d, 0.0],
                      [dy / d ** 2, -dx / d ** 2, -1.0]])
        return predicted, H

    def update(self, marker):
        """Fuse one marker's range and bearing. Returns (accepted, mahalanobis^2)."""
        if not self.initialised or marker.bearing is None:
            return False, None
        predicted, H = self.measurement(marker)
        innovation = np.array([marker.range - predicted[0], wrap(marker.bearing - predicted[1])])
        sigma_r = self.noise.sigma_r(predicted[0], marker.side_px)
        R = np.diag([sigma_r ** 2, self.noise.sigma_b ** 2])
        S = H @ self.P @ H.T + R
        S_inv = np.linalg.inv(S)
        distance2 = float(innovation @ S_inv @ innovation)
        if distance2 > self.gate:
            self.rejected_in_row += 1
            self.last_rejection = (f'{marker.label}: range off {innovation[0]:+.2f} m, bearing off '
                                   f'{math.degrees(innovation[1]):+.1f} deg (m2={distance2:.1f})')
            return False, distance2
        K = self.P @ H.T @ S_inv
        self.state = self.state + K @ innovation
        self.state[2] = wrap(self.state[2])
        I_KH = np.eye(3) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T      # Joseph form: stays symmetric
        self.accepted += 1
        self.rejected_in_row = 0
        return True, distance2

    # ------------------------------------------------------------ output
    def as_dict(self):
        if not self.initialised:
            return {'initialised': False}
        sx, sy, sh = (math.sqrt(max(v, 0.0)) for v in np.diag(self.P))
        return {'initialised': True,
                'x_m': round(float(self.state[0]), 3), 'y_m': round(float(self.state[1]), 3),
                'heading_deg': round(math.degrees(self.state[2]), 1),
                'sigma_x_m': round(sx, 3), 'sigma_y_m': round(sy, 3),
                'sigma_heading_deg': round(math.degrees(sh), 2),
                'updates_accepted': self.accepted, 'rejected_in_row': self.rejected_in_row,
                'last_rejection': self.last_rejection}
