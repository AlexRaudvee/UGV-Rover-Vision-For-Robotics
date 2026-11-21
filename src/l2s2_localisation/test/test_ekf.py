"""Odometry + marker EKF tests (simulated drives, no ROS)."""

import math

import numpy as np
import pytest

from l2s2_localisation.ekf import MarkerEKF, MotionNoise, relative_motion
from l2s2_localisation.triangulation import Landmark, wrap
from l2s2_localisation.triangulation_advanced import NoiseModel

MARKERS = {'m2': (0.0, -0.75), 'm3': (0.0, 0.75), 'm4': (4.5, 2.0)}
FOCAL, SIZE = 289.1, 0.14


def see(pose, names, rng=None, bearing_sigma=0.0):
    """Range + bearing of the named markers from a true pose."""
    out = []
    for name in names:
        x, y = MARKERS[name]
        d = math.hypot(x - pose[0], y - pose[1])
        side = FOCAL * SIZE / d
        r = d + (rng.normal(0, d * 0.5 / side) if rng is not None else 0.0)
        b = math.atan2(y - pose[1], x - pose[0]) - pose[2]
        b += rng.normal(0, bearing_sigma) if rng is not None else 0.0
        out.append(Landmark(name, (x, y), r, wrap(b), side))
    return out


def drive(steps=200, speed=0.02, turn=0.004, start=(-3.0, 0.0, 0.0)):
    """True poses of a gentle curve towards the markers (2 cm per step)."""
    poses = [np.array(start, float)]
    for _ in range(steps):
        x, y, h = poses[-1]
        poses.append(np.array([x + speed * math.cos(h), y + speed * math.sin(h), wrap(h + turn)]))
    return poses


def test_relative_motion_round_trip():
    a, b = (1.0, 2.0, 0.7), (1.3, 2.4, 1.1)
    dx, dy, dh = relative_motion(a, b)
    ekf = MarkerEKF()
    ekf.initialise(a, 0.01, 0.1)
    ekf.predict(dx, dy, dh)
    assert ekf.state == pytest.approx(b, abs=1e-9)


def test_motion_jacobian_matches_numerical_slopes():
    ekf = MarkerEKF(motion=MotionNoise(0, 0, 0, 0, 0))
    base = np.array([0.4, -0.2, 0.9])
    step = (0.3, 0.05, 0.1)
    ekf.initialise(base)
    ekf.P = np.zeros((3, 3))
    ekf.P[2, 2] = 1.0                      # P after predict = F e3 e3^T F^T: column 3 of F
    ekf.predict(*step)
    numeric = []
    for k in range(3):
        moved = base.copy()
        moved[k] += 1e-6
        e = MarkerEKF()
        e.initialise(moved)
        e.predict(*step)
        numeric.append((e.state - ekf.state) / 1e-6)
    column = np.array(numeric)[2]          # derivative wrt heading
    assert np.allclose(ekf.P[:, 2] / ekf.P[2, 2], column, atol=1e-5)


def test_initialise_from_two_markers():
    ekf = MarkerEKF()
    fix = ekf.initialise_from_markers(see((-2.0, 0.5, 0.2), ['m2', 'm3']))
    assert fix is not None
    assert ekf.state == pytest.approx((-2.0, 0.5, 0.2), abs=1e-6)


def test_one_marker_update_shrinks_uncertainty():
    ekf = MarkerEKF()
    ekf.initialise((-2.2, 0.3, 0.1), sigma_xy=0.3, sigma_h_deg=10)
    before = np.trace(ekf.P)
    accepted, _ = ekf.update(see((-2.0, 0.5, 0.2), ['m3'])[0])
    assert accepted
    assert np.trace(ekf.P) < before


def test_outlier_is_rejected():
    ekf = MarkerEKF()
    ekf.initialise((-2.0, 0.5, 0.2), sigma_xy=0.05, sigma_h_deg=2)
    wrong = Landmark('m3', MARKERS['m3'], 6.0, 1.0, 10.0)      # 6 m away, far off bearing
    accepted, distance2 = ekf.update(wrong)
    assert not accepted and distance2 > ekf.gate
    assert ekf.state == pytest.approx((-2.0, 0.5, 0.2))


def test_estimate_inside_reported_uncertainty():
    rng = np.random.default_rng(11)
    inside, runs = 0, 30
    for run in range(runs):
        truth = drive(steps=100)
        ekf = MarkerEKF(noise=NoiseModel(sigma_pan_deg=1.0))
        ekf.initialise_from_markers(see(truth[0], ['m2', 'm3'], rng, math.radians(1)))
        for k in range(1, len(truth)):
            dx, dy, dh = relative_motion(truth[k - 1], truth[k])
            ekf.predict(dx * (1 + rng.normal(0, 0.03)), dy, dh + rng.normal(0, 0.003))
            if k % 4 == 0:
                for marker in see(truth[k], ['m2', 'm3'], rng, math.radians(1)):
                    ekf.update(marker)
        error = ekf.state[:2] - truth[-1][:2]
        inside += float(error @ np.linalg.inv(ekf.P[:2, :2]) @ error) < 9.21   # 99 % ellipse
    assert inside >= 0.8 * runs
