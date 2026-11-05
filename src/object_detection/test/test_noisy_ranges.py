"""Noise, isolated spikes, missing samples, and tolerance are independent regressions."""

import math
from types import SimpleNamespace

import numpy as np
from object_detection.perception import (
    DetectorSettings, estimate_surface, measure_blob, RangeEstimate, validate,
)
from object_detection.ranging import TemporalRanges
import pytest


def test_near_speckles_do_not_override_a_supported_surface():
    values = np.concatenate((np.full(20, .3), np.linspace(1.45, 1.55, 1000), np.full(200, 4.)))
    estimate = estimate_surface(values, 15, DetectorSettings())
    assert estimate.axial_m == pytest.approx(1.5)
    assert estimate.samples >= 900
    assert .01 <= estimate.std_m < .1


def test_broad_bridged_depth_surface_is_unknown():
    estimate = estimate_surface(np.linspace(.3, 5., 1000), 15, DetectorSettings())
    assert math.isnan(estimate.axial_m)


def test_temporal_filter_suppresses_jitter_and_rejects_isolated_jumps():
    settings = DetectorSettings()
    blob = SimpleNamespace(bbox=(10, 10, 40, 60))
    tracker = TemporalRanges()

    def update(distance, now):
        return tracker.update([blob], [RangeEstimate(distance, .01, 100, 1.)], now, settings)[0]

    assert update(1.50, 1.).axial_m == 1.5
    assert update(1.54, 1.1).axial_m == pytest.approx(1.52)
    assert update(1.48, 1.2).axial_m == 1.5
    assert math.isnan(update(3.0, 1.3).axial_m)
    assert update(1.51, 1.4).axial_m < 1.55
    assert math.isnan(update(math.nan, 1.5).axial_m)
    assert update(2., 3.).axial_m == 2.  # Expired track cannot retain the old surface.


def test_persistent_range_change_is_adopted_without_blending_surfaces():
    tracker, settings = TemporalRanges(), DetectorSettings()
    blob = SimpleNamespace(bbox=(10, 10, 40, 60))
    for timestamp, distance in [(1., 1.), (1.1, 2.), (1.2, 2.02)]:
        value = tracker.update([blob], [RangeEstimate(distance, .01, 100, 1.)],
                               timestamp, settings)[0]
    assert value.axial_m == 2.02


def test_width_tolerance_is_bounded_and_labels_borderline_measurements():
    blob = SimpleNamespace(bbox=(10, 10, 100, 150), clipped=False, bearing=0.,
                           ray_span=.36, angle_min=-.18, angle_max=.18, area_px=1000)
    estimate = RangeEstimate(.85, .01, 50, 1.)
    item = measure_blob(blob, .85, 'lidar', DetectorSettings(), estimate)
    assert item.width_m == pytest.approx(.306)
    assert item.accepted and not item.nominal_width
    assert not measure_blob(blob, 1., 'lidar', DetectorSettings(), estimate).accepted


def ema_update(tracker, distance, timestamp, settings, bbox=(10, 10, 40, 60)):
    """Feed one independently supported depth sample into a matching silhouette."""
    return tracker.update([SimpleNamespace(bbox=bbox)],
                          [RangeEstimate(distance, .01, 100, 1.)], timestamp, settings)[0]


def test_depth_ema_uses_elapsed_time_and_exposes_raw_measurement():
    settings = DetectorSettings(depth_ema_tau_s=.2 / math.log(2))
    tracker = TemporalRanges(use_ema=True)
    first = ema_update(tracker, 1., 1., settings)
    second = ema_update(tracker, 1.1, 1.2, settings)
    third = ema_update(tracker, 1.1, 1.6, settings)
    assert first.axial_m == 1. and first.ema_alpha == 1.
    assert second.ema_alpha == pytest.approx(.5)
    assert second.axial_m == pytest.approx(1.05)
    assert second.raw_axial_m == 1.1 and second.filter_method == 'ema'
    assert third.ema_alpha == pytest.approx(.75)
    assert third.axial_m == pytest.approx(1.0875)
    assert second.std_m >= abs(second.raw_axial_m - second.axial_m)


def test_depth_ema_reduces_independent_stationary_jitter():
    settings, tracker = DetectorSettings(), TemporalRanges(use_ema=True)
    samples = 1.5 + np.random.default_rng(7).normal(0, .03, 100)
    smoothed = [ema_update(tracker, float(value), 1. + index * .2, settings).axial_m
                for index, value in enumerate(samples)]
    assert np.std(smoothed[10:]) < .7 * np.std(samples[10:])
    assert abs(np.mean(smoothed[10:]) - np.mean(samples[10:])) < .01


@pytest.mark.parametrize('invalid', [math.nan, 0., math.inf])
def test_depth_ema_never_fills_missing_or_zero_samples(invalid):
    settings, tracker = DetectorSettings(), TemporalRanges(use_ema=True)
    ema_update(tracker, 1.5, 1., settings)
    assert math.isnan(ema_update(tracker, invalid, 1.2, settings).axial_m)
    # Invalid callbacks keep the silhouette alive but must not keep its EMA alive.
    ema_update(tracker, invalid, 1.4, settings)
    ema_update(tracker, invalid, 1.6, settings)
    assert ema_update(tracker, 1.7, 1.8, settings).axial_m == 1.7


def test_depth_ema_resets_after_confirmed_surface_jump_or_clock_rewind():
    settings, tracker = DetectorSettings(), TemporalRanges(use_ema=True)
    ema_update(tracker, 1., 1., settings)
    assert math.isnan(ema_update(tracker, 2., 1.2, settings).axial_m)
    assert ema_update(tracker, 2.02, 1.4, settings).axial_m == 2.02
    assert ema_update(tracker, .8, .5, settings).axial_m == .8


def test_depth_ema_does_not_mix_different_silhouettes():
    settings, tracker = DetectorSettings(), TemporalRanges(use_ema=True)
    ema_update(tracker, 1., 1., settings)
    assert ema_update(tracker, 2., 1.2, settings, bbox=(100, 10, 40, 60)).axial_m == 2.
    assert ema_update(tracker, 1.1, 1.4, settings).axial_m < 1.1


def test_depth_ema_can_be_disabled_and_rejects_invalid_time_constants():
    settings, tracker = DetectorSettings(depth_ema_tau_s=0.), TemporalRanges(use_ema=True)
    validate(settings)
    ema_update(tracker, 1., 1., settings)
    value = ema_update(tracker, 1.1, 1.2, settings)
    assert value.axial_m == 1.1 and value.filter_method == 'none'
    for tau in [-.1, math.nan, math.inf, 3.]:
        with pytest.raises(ValueError):
            validate(DetectorSettings(depth_ema_tau_s=tau))
