"""Advanced (range + bearing) localization tests."""

import math

import numpy as np
import pytest

from l2s2_localisation.triangulation import Landmark, localize, wrap
from l2s2_localisation.triangulation_advanced import (NoiseModel, build_system,
                                                        localize_advanced, predict)

SKETCH = [(0.0, 3.9), (0.0, -3.9), (-5.4, 0.0)]       # layout of the hand sketch
FOCAL, SIZE = 289.1, 0.25


def measure(robot, heading, markers, range_noise=False, bearing_sigma=0.0, seed=0):
    """Simulated scan: range noise grows with distance^2, as for the real camera."""
    rng = np.random.default_rng(seed)
    out = []
    for index, (x, y) in enumerate(markers):
        d = math.hypot(x - robot[0], y - robot[1])
        side = FOCAL * SIZE / d
        r = d + (rng.normal(0, d * 0.5 / side) if range_noise else 0.0)
        b = wrap(math.atan2(y - robot[1], x - robot[0]) - heading
                 + rng.normal(0, bearing_sigma))
        out.append(Landmark(f'm{index}', (x, y), r, b, side))
    return out


def test_jacobian_rows_match_numerical_slopes():
    """Move the robot by a tiny step, recompute the prediction, divide the change."""
    markers = measure((0.4, -0.7), 0.3, SKETCH)
    state = np.array([0.5, -0.6, 0.2])
    _, jacobian, _ = build_system(state, markers, NoiseModel())
    step = 1e-6
    numeric = []
    for marker in markers:
        for part in (2, 3):            # di then ai
            row = []
            for k in range(3):
                moved = state.copy()
                moved[k] += step
                change = predict(moved, marker)[part] - predict(state, marker)[part]
                row.append(wrap(change) / step if part == 3 else change / step)
            numeric.append(row)
    assert np.allclose(jacobian, numeric, atol=1e-5)


def test_exact_measurements_give_exact_pose():
    fix = localize_advanced(measure((-1.2, 0.8), -0.7, SKETCH))
    assert (fix.x, fix.y) == pytest.approx((-1.2, 0.8), abs=1e-6)
    assert fix.h == pytest.approx(-0.7, abs=1e-6)


def test_two_markers_are_enough():
    markers = measure((-1.0, 0.3), 0.4, [(0.0, 1.0), (0.0, -1.0)])
    fix = localize_advanced(markers)
    assert (fix.x, fix.y) == pytest.approx((-1.0, 0.3), abs=1e-6)


def test_heading_near_180_degrees():
    fix = localize_advanced(measure((1.0, 0.5), math.radians(179.5), SKETCH))
    assert abs(wrap(fix.h - math.radians(179.5))) < 1e-6


def test_better_than_ranges_only_with_realistic_noise():
    rng = np.random.default_rng(3)
    basic_errors, advanced_errors, inside_1_sigma = [], [], 0
    for seed in range(60):
        robot = (rng.uniform(-2, 2), rng.uniform(-2, 2))
        heading = rng.uniform(-math.pi, math.pi)
        markers = measure(robot, heading, SKETCH, range_noise=True,
                          bearing_sigma=math.radians(1.0), seed=seed)
        basic = localize(markers).position
        advanced = localize_advanced(markers, NoiseModel(sigma_pan_deg=1.0))
        basic_errors.append(math.dist(basic, robot))
        advanced_errors.append(math.dist((advanced.x, advanced.y), robot))
        sx, sy = advanced.sigma_xy
        inside_1_sigma += abs(advanced.x - robot[0]) < sx and abs(advanced.y - robot[1]) < sy
    assert np.median(advanced_errors) < 0.7 * np.median(basic_errors)
    assert inside_1_sigma > 0.25 * 60          # reported uncertainty is not wildly off


def test_needs_bearings():
    markers = [Landmark('a', (0, 1), 1.0), Landmark('b', (0, -1), 1.0)]
    with pytest.raises(ValueError):
        localize_advanced(markers)


def test_uncertainty_grows_with_pan_error():
    markers = measure((0.5, 0.5), 0.0, SKETCH)
    tight = localize_advanced(markers, NoiseModel(sigma_pan_deg=0.1)).sigma_xy
    loose = localize_advanced(markers, NoiseModel(sigma_pan_deg=3.0)).sigma_xy
    assert loose[0] > tight[0] and loose[1] > tight[1]
