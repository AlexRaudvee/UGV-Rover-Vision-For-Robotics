"""Triangulation: robot position from floor ranges to markers at known positions.

Follows the hand derivation ("Proof of triangulation"):

  circle i:              (x - x_i)^2 + (y - y_i)^2 = r_i^2
  circle 2 - circle 1:   A x + B y = C
                         A = 2 (x2 - x1),  B = 2 (y2 - y1)
                         C = r1^2 - r2^2 + (x2^2 + y2^2) - (x1^2 + y1^2)
  substitute y = (C - A x) / B into circle 1 (times B^2), with k = C - B y1:
                         alpha x^2 + beta x + gamma = 0
                         alpha = A^2 + B^2
                         beta  = -2 (B^2 x1 + A k)
                         gamma = B^2 x1^2 + k^2 - B^2 r1^2
                         x = (-beta +/- sqrt(beta^2 - 4 alpha gamma)) / (2 alpha)

Additions needed in practice:
  * |B| small: dividing by B is unstable, so when |A| > |B| the same steps are
    done with x and y swapped (solve x = (C - B y) / A).
  * beta^2 - 4 alpha gamma < 0: noisy ranges, the circles miss each other.
    x = -beta / (2 alpha) is then the point on the line between the centres
    where the circles come closest; the fix is flagged.
  * Two solutions (Fig. 3): drop candidates outside the field, then use the
    measured bearings, whose differences do not depend on the robot heading.
  * Three or more markers: stack "circle i - circle 1" rows A_i x + B_i y = C_i
    and solve by least squares, then refine on the original circle equations
    (Gauss-Newton), because the subtraction mixes r1's noise into every row.

Field frame (Table 2 / Fig. 7): origin at the centre spot, x towards the goal
with the targets, y to the left, metres. No ROS imports.
"""

from dataclasses import dataclass, field
import math

import numpy as np


@dataclass(frozen=True)
class Field:
    """Soccer field (Fig. 7): 9 x 6 m plus a 0.7 m border strip."""

    half_length: float = 4.5
    half_width: float = 3.0
    border: float = 0.7

    def contains(self, point):
        return (abs(point[0]) <= self.half_length + self.border
                and abs(point[1]) <= self.half_width + self.border)


@dataclass
class Landmark:
    """A marker with a known field position and a measured range (and bearing)."""

    label: str
    position: tuple            # (x, y) in the field frame, metres
    range: float               # floor distance robot -> marker, metres
    bearing: float = None      # rad, relative to the robot heading, left positive
    side_px: float = None      # marker size in the image; sets the range noise (advanced)


@dataclass
class Fix:
    position: np.ndarray
    heading: float = None      # rad in the field frame, None without bearings
    method: str = ''
    candidates: list = field(default_factory=list)
    rms_range_error: float = 0.0
    warnings: list = field(default_factory=list)

    def as_dict(self):
        return {
            'x_m': round(float(self.position[0]), 3), 'y_m': round(float(self.position[1]), 3),
            'heading_deg': None if self.heading is None else round(math.degrees(self.heading), 1),
            'method': self.method, 'rms_range_error_m': round(self.rms_range_error, 4),
            'candidates': [[round(float(c[0]), 3), round(float(c[1]), 3)]
                           for c in self.candidates],
            'warnings': self.warnings,
        }


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------- two markers
def radical_line(p1, r1, p2, r2):
    """Circle 2 - circle 1 gives the line A x + B y = C through both intersections."""
    (x1, y1), (x2, y2) = p1, p2
    a = 2.0 * (x2 - x1)
    b = 2.0 * (y2 - y1)
    c = r1 ** 2 - r2 ** 2 + (x2 ** 2 + y2 ** 2) - (x1 ** 2 + y1 ** 2)
    return a, b, c


def _quadratic_roots(a, b, c, u1, v1, r1):
    """Roots u of alpha u^2 + beta u + gamma = 0 after substituting v = (C - A u) / B.

    With (u, v) = (x, y) these are exactly the hand-derived alpha, beta, gamma.
    Returns (roots, circles_meet).
    """
    k = c - b * v1
    alpha = a ** 2 + b ** 2
    beta = -2.0 * (b ** 2 * u1 + a * k)
    gamma = b ** 2 * u1 ** 2 + k ** 2 - b ** 2 * r1 ** 2
    discriminant = beta ** 2 - 4.0 * alpha * gamma
    if discriminant < 0.0:
        return [-beta / (2.0 * alpha)], False
    root = math.sqrt(discriminant)
    return [(-beta + root) / (2.0 * alpha), (-beta - root) / (2.0 * alpha)], True


def circle_intersections(p1, r1, p2, r2):
    """Both intersection points of two range circles (one point if they do not meet).

    Returns (candidates, circles_meet).
    """
    a, b, c = radical_line(p1, r1, p2, r2)
    if a == 0.0 and b == 0.0:
        raise ValueError('Both markers are at the same position')
    (x1, y1) = p1
    if abs(b) >= abs(a):
        # As derived: y = (C - A x) / B, quadratic in x.
        xs, meet = _quadratic_roots(a, b, c, x1, y1, r1)
        points = [np.array([x, (c - a * x) / b]) for x in xs]
    else:
        # Same steps with x and y swapped: x = (C - B y) / A, quadratic in y.
        ys, meet = _quadratic_roots(b, a, c, y1, x1, r1)
        points = [np.array([(c - b * y) / a, y]) for y in ys]
    return points, meet


# ---------------------------------------------------------------- bearings
def heading_and_bearing_error(position, landmarks):
    """Heading that best explains the bearings, and the remaining RMS bearing error.

    Seen from the robot, marker i lies at field angle theta_i; the robot measured
    it at bearing b_i, so heading = theta_i - b_i for every marker. The wrong
    (mirrored) candidate sees the markers in the opposite order, so these
    estimates disagree and the error is large.
    """
    seen = [m for m in landmarks if m.bearing is not None]
    if not seen:
        return None, None
    estimates = [math.atan2(m.position[1] - position[1], m.position[0] - position[0])
                 - m.bearing for m in seen]
    heading = math.atan2(sum(math.sin(e) for e in estimates),
                         sum(math.cos(e) for e in estimates))
    error = math.sqrt(sum(wrap(e - heading) ** 2 for e in estimates) / len(estimates))
    return heading, error


def choose_candidate(candidates, landmarks, field_model):
    """Pick the real position: inside the field, then consistent with the bearings."""
    inside = [c for c in candidates if field_model.contains(c)]
    pool = inside or candidates
    if len(pool) == 1:
        return pool[0], 'field' if len(inside) < len(candidates) else 'single'
    scored = [(heading_and_bearing_error(c, landmarks)[1], i) for i, c in enumerate(pool)]
    if scored[0][0] is None:
        return pool[0], 'ambiguous'
    return pool[min(scored)[1]], 'bearing'


# ---------------------------------------------------------------- three or more
def linear_least_squares(landmarks):
    """Rows 'circle i - circle 1': A_i x + B_i y = C_i, solved by least squares.

    Returns None when the markers are (nearly) collinear: the rows then only
    fix the position along one direction and the mirror ambiguity remains.
    """
    first = landmarks[0]
    rows, rhs = [], []
    for other in landmarks[1:]:
        a, b, c = radical_line(first.position, first.range, other.position, other.range)
        rows.append([a, b])
        rhs.append(c)
    matrix = np.array(rows)
    singular = np.linalg.svd(matrix, compute_uv=False)
    if len(singular) < 2 or singular[1] < 1e-6 * singular[0]:
        return None
    solution, *_ = np.linalg.lstsq(matrix, np.array(rhs), rcond=None)
    return solution


def refine(position, landmarks, iterations=10):
    """Gauss-Newton on the original equations: minimize sum (|X - P_i| - r_i)^2."""
    position = np.array(position, float)
    points = np.array([m.position for m in landmarks], float)
    ranges = np.array([m.range for m in landmarks], float)
    for _ in range(iterations):
        offsets = position - points
        distances = np.maximum(np.linalg.norm(offsets, axis=1), 1e-9)
        jacobian = offsets / distances[:, None]
        step, *_ = np.linalg.lstsq(jacobian, ranges - distances, rcond=None)
        position = position + step
        if np.linalg.norm(step) < 1e-9:
            break
    return position


def rms_range_error(position, landmarks):
    errors = [math.hypot(position[0] - m.position[0], position[1] - m.position[1]) - m.range
              for m in landmarks]
    return math.sqrt(sum(e * e for e in errors) / len(errors))


# ---------------------------------------------------------------- entry point
def localize(landmarks, field_model=Field(), use_refine=True):
    """Robot position (and heading if bearings are given) from 2+ landmarks."""
    landmarks = list(landmarks)
    if len(landmarks) < 2:
        raise ValueError('Triangulation needs at least two markers with known positions')
    warnings = []

    if len(landmarks) >= 3:
        linear = linear_least_squares(landmarks)
        if linear is not None:
            position = refine(linear, landmarks) if use_refine else linear
            method = f'least squares, {len(landmarks)} markers' + (
                ' + Gauss-Newton' if use_refine else '')
            heading, _ = heading_and_bearing_error(position, landmarks)
            if not field_model.contains(position):
                warnings.append('position outside the field')
            return Fix(position, heading, method, [linear], rms_range_error(position, landmarks),
                       warnings)
        warnings.append('markers are collinear: using the two farthest apart')

    # Two markers (or collinear ones): intersect the two circles farthest apart.
    pairs = [(i, j) for i in range(len(landmarks)) for j in range(i + 1, len(landmarks))]
    i, j = max(pairs, key=lambda p: math.dist(landmarks[p[0]].position,
                                              landmarks[p[1]].position))
    candidates, meet = circle_intersections(landmarks[i].position, landmarks[i].range,
                                            landmarks[j].position, landmarks[j].range)
    if not meet:
        warnings.append('range circles do not intersect (noise): closest point used')
    position, reason = choose_candidate(candidates, landmarks, field_model)
    if reason == 'ambiguous':
        warnings.append('two solutions inside the field and no bearings: first one used')
    if use_refine and len(landmarks) > 2:
        position = refine(position, landmarks)
    heading, _ = heading_and_bearing_error(position, landmarks)
    return Fix(position, heading, f'two circles, chosen by {reason}', candidates,
               rms_range_error(position, landmarks), warnings)
