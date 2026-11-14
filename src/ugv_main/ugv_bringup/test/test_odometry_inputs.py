"""Regression checks for feedback resets, gaps and invalid wheel samples."""

import math

from ugv_bringup.odometry_inputs import WheelVelocity


def test_first_sample_is_only_a_baseline():
    wheels = WheelVelocity()
    assert wheels.update([120.0, 150.0], 10.0) is None
    assert math.isclose(wheels.update([120.01, 150.01], 10.1), 0.1)


def test_reset_and_gap_rebase_without_false_motion():
    wheels = WheelVelocity()
    wheels.update([100.0, 100.0], 1.0)
    assert wheels.update([0.0, 0.0], 1.1) is None
    assert math.isclose(wheels.update([0.01, 0.01], 1.2), 0.1)
    assert wheels.update([3.0, 3.0], 3.0) is None
    assert math.isclose(wheels.update([3.01, 3.01], 3.1), 0.1)


def test_invalid_time_or_values_never_create_velocity():
    wheels = WheelVelocity()
    assert wheels.update([float('nan'), 0.0], 1.0) is None
    assert wheels.update([], 1.0) is None
    assert wheels.update([0.0, 0.0], 1.0) is None
    assert wheels.update([0.0, 0.0], 1.0) is None
    assert wheels.update([0.0, 0.0], 0.5) is None
    assert wheels.update([0.0, float('inf')], 1.0) is None


def test_turn_in_place_does_not_require_wheel_separation():
    wheels = WheelVelocity()
    wheels.update([0.0, 0.0], 1.0)
    assert wheels.update([-0.01, 0.01], 1.1) == 0.0
