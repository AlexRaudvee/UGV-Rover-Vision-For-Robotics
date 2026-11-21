"""Odometry + marker EKF tests (simulated drives, no ROS)."""

import math

import numpy as np
import pytest

from l2s2_localisation.ekf import MarkerEKF, MotionNoise, map_to_odom, relative_motion
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


@pytest.mark.parametrize('odom,map_pose', [
    ((1., 2., .7), (-3.2, 0., 0.)),
    ((8., -4., -3.1), (2., .5, 3.1)),
    ((0., 0., 0.), (.4, -.2, 1.7)),
])
def test_map_to_odom_composes_back_to_corrected_robot_pose(odom, map_pose):
    from l2s2_localisation.ranging import planar_pose
    actual = planar_pose(map_to_odom(map_pose, odom), odom)
    assert actual[:2] == pytest.approx(map_pose[:2])
    assert wrap(actual[2]-map_pose[2]) == pytest.approx(0.)


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


def biased_drive(seed, bias_deg_per_step, visible=('m3',), **ekf_options):
    """4 m drive; odometry has 3 % scale error and a heading bias.

    Returns (EKF error, odometry-only error, ekf) at the end of the drive."""
    rng = np.random.default_rng(seed)
    truth = drive()
    ekf = MarkerEKF(noise=NoiseModel(sigma_pan_deg=1.0), **ekf_options)
    ekf.initialise_from_markers(see(truth[0], ['m2', 'm3'], rng, math.radians(1)))
    dead = np.array(ekf.state)
    for k in range(1, len(truth)):
        dx, dy, dh = relative_motion(truth[k - 1], truth[k])
        noisy = (dx * 1.03, dy, dh + math.radians(bias_deg_per_step) + rng.normal(0, 0.002))
        ekf.predict(*noisy)
        c, s = math.cos(dead[2]), math.sin(dead[2])
        dead = np.array([dead[0] + c * noisy[0] - s * noisy[1],
                         dead[1] + s * noisy[0] + c * noisy[1], wrap(dead[2] + noisy[2])])
        if k % 5 == 0:                       # one camera frame every 10 cm driven
            for marker in see(truth[k], visible, rng, math.radians(1)):
                ekf.update(marker)
    return (math.dist(ekf.state[:2], truth[-1][:2]), math.dist(dead[:2], truth[-1][:2]), ekf)


SEEDS = range(10)


def test_one_marker_beats_dead_reckoning():
    """Realistic drift (1.5 deg per metre), only ONE marker ever visible."""
    runs = [biased_drive(seed, 0.03) for seed in SEEDS]
    ekf_errors = [r[0] for r in runs]
    dead_errors = [r[1] for r in runs]
    assert np.median(ekf_errors) < 0.15
    assert np.median(ekf_errors) < 0.5 * np.median(dead_errors)


def test_two_markers_are_better_than_one():
    one = np.median([biased_drive(seed, 0.03)[0] for seed in SEEDS])
    two = np.median([biased_drive(seed, 0.03, visible=('m2', 'm3'))[0] for seed in SEEDS])
    assert two < one and two < 0.1


def test_recovers_from_odometry_worse_than_modelled():
    """7.5 deg per metre drift, far beyond MotionNoise: without recovery the gate
    locks out the correct markers."""
    runs = [biased_drive(seed, 0.15) for seed in SEEDS]
    assert sum(r[2].recoveries for r in runs) > 0
    recovered = np.median([r[0] for r in runs])
    assert recovered < 0.5 * np.median([r[1] for r in runs])
    locked = np.median([biased_drive(seed, 0.15, recover_after=0)[0] for seed in SEEDS])
    assert recovered < locked


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
