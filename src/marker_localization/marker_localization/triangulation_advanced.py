"""Advanced localization: ranges AND bearings, weighted by how noisy each one is.

Equations (same names as the write-up):

  1. prediction from a guess (x, y, h):
         dxi = xi - x,   dyi = yi - y
         di  = sqrt(dxi^2 + dyi^2)               predicted range
         ai  = atan2(dyi, dxi) - h               predicted bearing
  2. errors:
         ei = ri - di                            range error
         gi = wrap(bi - ai)                      bearing error, kept in -180..180 deg
  3. noise (how much to trust each sensor):
         sigma_r = di * sigma_px / side_px       (= d^2 sigma_px / (f S))
         sigma_b = sqrt((sigma_px / f)^2 + sigma_pan^2)
  4. score = sum (ei / sigma_r)^2 + sum (gi / sigma_b)^2     -> minimise
  5. start: (x0, y0) from the two-circle method (triangulation.py),
            h0 = circular mean of (atan2(yi - y0, xi - x0) - bi)
  6. improve, 5-10 times:
         range row   [ -dxi/di,    -dyi/di,     0 ]
         bearing row [  dyi/di^2,  -dxi/di^2,  -1 ]
         (J^T W J) delta = J^T W e,   (x, y, h) += delta
  7. uncertainty = (J^T W J)^-1

Works with 2 or more markers that have bearings. No ROS imports.
"""

from dataclasses import dataclass, field
import math

import numpy as np

from .triangulation import Field, localize, wrap


@dataclass(frozen=True)
class NoiseModel:
    """Sensor noise. Measure sigma_pan_deg on the rover (repeat scans of one marker)."""

    sigma_px: float = 0.5          # corner noise in pixels
    sigma_pan_deg: float = 1.0     # pan servo error (no feedback, so unknown until measured)
    focal_px: float = 289.1        # fx of the USB camera (camera_info.yaml)
    min_sigma_r: float = 0.01      # never trust a range better than 1 cm

    @property
    def sigma_b(self):
        return math.sqrt((self.sigma_px / self.focal_px) ** 2
                         + math.radians(self.sigma_pan_deg) ** 2)

    def sigma_r(self, distance, side_px):
        if side_px is None or side_px <= 0:
            return max(self.min_sigma_r, 0.05 * distance)   # no size info: assume 5 %
        return max(self.min_sigma_r, distance * self.sigma_px / side_px)


@dataclass
class AdvancedFix:
    x: float
    y: float
    h: float                          # heading, rad
    covariance: np.ndarray            # 3x3 for (x, y, h)
    iterations: int
    start: tuple                      # (x0, y0, h0) from the two-circle method
    score: float
    warnings: list = field(default_factory=list)

    @property
    def sigma_xy(self):
        """1-sigma uncertainty in x and y, metres."""
        return tuple(math.sqrt(max(v, 0.0)) for v in np.diag(self.covariance)[:2])

    def as_dict(self):
        sx, sy = self.sigma_xy
        return {'x_m': round(self.x, 3), 'y_m': round(self.y, 3),
                'heading_deg': round(math.degrees(self.h), 1),
                'sigma_x_m': round(sx, 3), 'sigma_y_m': round(sy, 3),
                'sigma_heading_deg': round(math.degrees(math.sqrt(
                    max(self.covariance[2, 2], 0.0))), 2),
                'iterations': self.iterations, 'score': round(self.score, 3),
                'start': [round(v, 3) for v in self.start], 'warnings': self.warnings,
                'method': 'range + bearing, weighted least squares'}


# ---------------------------------------------------------------- equations 1-3, 6
def predict(state, marker):
    """Step 1: predicted range di and bearing ai for marker i from state (x, y, h)."""
    x, y, h = state
    dxi, dyi = marker.position[0] - x, marker.position[1] - y
    di = math.hypot(dxi, dyi)
    ai = math.atan2(dyi, dxi) - h
    return dxi, dyi, di, ai


def build_system(state, markers, noise):
    """Errors e, Jacobian J and weights W for the current guess (steps 2, 3 and 6)."""
    errors, rows, weights = [], [], []
    for marker in markers:
        dxi, dyi, di, ai = predict(state, marker)
        di = max(di, 1e-6)
        sigma_r = noise.sigma_r(di, marker.side_px)
        errors.append(marker.range - di)                              # ei
        rows.append([-dxi / di, -dyi / di, 0.0])                      # range row
        weights.append(1.0 / sigma_r ** 2)
        if marker.bearing is not None:
            errors.append(wrap(marker.bearing - ai))                  # gi
            rows.append([dyi / di ** 2, -dxi / di ** 2, -1.0])        # bearing row
            weights.append(1.0 / noise.sigma_b ** 2)
    return np.array(errors), np.array(rows), np.array(weights)


def score(state, markers, noise):
    """Step 4: sum of squared errors, each divided by its noise."""
    errors, _, weights = build_system(state, markers, noise)
    return float(np.sum(weights * errors ** 2))


# ---------------------------------------------------------------- step 5
def starting_guess(markers, field_model):
    """Two-circle / least-squares position, heading from the bearings."""
    basic = localize(markers, field_model)
    x0, y0 = float(basic.position[0]), float(basic.position[1])
    h0 = basic.heading if basic.heading is not None else 0.0
    return (x0, y0, h0), basic.warnings


# ---------------------------------------------------------------- main
def localize_advanced(markers, noise=NoiseModel(), field_model=Field(), max_iterations=20):
    """Robot (x, y, heading) from ranges and bearings of 2+ known markers."""
    markers = list(markers)
    if len(markers) < 2:
        raise ValueError('Need at least two markers with known positions')
    if sum(m.bearing is not None for m in markers) < 2:
        raise ValueError('The advanced method needs bearings for at least two markers; '
                         'use triangulation.localize for ranges only')

    start, warnings = starting_guess(markers, field_model)
    state = np.array(start, float)
    iterations = 0
    for iterations in range(1, max_iterations + 1):
        errors, jacobian, weights = build_system(state, markers, noise)
        jtw = jacobian.T * weights                       # J^T W
        delta = np.linalg.lstsq(jtw @ jacobian, jtw @ errors, rcond=None)[0]

        # Only accept steps that lower the score (halve the step otherwise).
        old = score(state, markers, noise)
        step = 1.0
        while step > 1e-4 and score(state + step * delta, markers, noise) > old:
            step /= 2.0
        state = state + step * delta
        state[2] = wrap(state[2])
        if np.linalg.norm(step * delta) < 1e-9:
            break

    _, jacobian, weights = build_system(state, markers, noise)
    information = (jacobian.T * weights) @ jacobian       # J^T W J
    if np.linalg.matrix_rank(information) < 3:
        warnings = warnings + ['geometry does not fix x, y and heading (rank < 3)']
        covariance = np.linalg.pinv(information)
    else:
        covariance = np.linalg.inv(information)            # step 7
    if not field_model.contains(state[:2]):
        warnings = warnings + ['position outside the field']
    return AdvancedFix(float(state[0]), float(state[1]), float(state[2]), covariance,
                       iterations, tuple(float(v) for v in start),
                       score(state, markers, noise), warnings)
