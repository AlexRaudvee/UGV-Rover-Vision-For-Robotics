"""Leave-one-out rejection of inconsistent markers (localize_robust)."""

import math

import numpy as np
import pytest

from l2s2_localisation.triangulation import Field, Landmark, wrap
from l2s2_localisation.triangulation_advanced import (NoiseModel, chi2_limit, localize_advanced,
                                                      localize_robust)

# Field data, 18 Nov 2026: rover at (0, 0), centre-spot sweep (center_sweep2_221736).
MAP = {1: (-1.5, -3.0), 2: (-1.5, 3.0), 3: (1.55, -3.0), 4: (1.55, 3.0), 5: (4.5, 0.0)}
CAMERA = {1: (3.42, -91.0, 8.8), 2: (3.51, 126.4, 8.2), 3: (3.57, -53.5, 8.2),
          4: (3.60, 72.2, 8.3), 5: (5.16, 6.8, 8.1)}          # range m, bearing deg, side px
FIELD = Field(4.5, 3.0, 0.7)
NOISE = NoiseModel(sigma_pan_deg=2.0)


def field_markers(ids):
    return [Landmark(f'ID{i}', MAP[i], CAMERA[i][0], math.radians(CAMERA[i][1]), CAMERA[i][2])
            for i in ids]


def test_chi2_limit_matches_tables():
    for dof, exact in ((1, 6.635), (3, 11.345), (5, 15.086), (7, 18.475)):
        assert chi2_limit(dof) == pytest.approx(exact, rel=0.02)


def test_field_data_drops_tag_1_and_recovers_the_position():
    plain = localize_advanced(field_markers([1, 2, 3, 4, 5]), NOISE, FIELD)
    fix, dropped = localize_robust(field_markers([1, 2, 3, 4, 5]), NOISE, FIELD)
    assert [label for label, _, _ in dropped] == ['ID1']
    assert math.hypot(fix.x, fix.y) < 0.1 < math.hypot(plain.x, plain.y)


def test_field_data_good_markers_are_all_kept():
    fix, dropped = localize_robust(field_markers([2, 3, 4, 5]), NOISE, FIELD)
    assert dropped == []
    assert math.hypot(fix.x, fix.y) < 0.1


def test_two_markers_are_never_dropped():
    fix, dropped = localize_robust(field_markers([1, 3]), NOISE, FIELD)
    assert dropped == []


def see(pose, markers, rng, sigma_b, sigma_r):
    out = []
    for label, (x, y) in markers.items():
        d = math.hypot(x - pose[0], y - pose[1])
        b = math.atan2(y - pose[1], x - pose[0]) - pose[2]
        out.append(Landmark(label, (x, y), d + rng.normal(0, sigma_r * d),
                            wrap(b + rng.normal(0, sigma_b)), 10.0))
    return out


def test_false_alarms_are_rare_with_honest_noise():
    """Good markers only, noise as modelled: a marker is dropped in about 1 % of fixes."""
    rng = np.random.default_rng(1)
    noise = NoiseModel(sigma_pan_deg=1.0)
    alarms = 0
    for _ in range(200):
        pose = (rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-math.pi, math.pi))
        markers = see(pose, {f'm{i}': MAP[i] for i in (2, 3, 4, 5)}, rng,
                      noise.sigma_b, 0.5 / 10.0)               # sigma_r = d * 0.5 px / 10 px
        _, dropped = localize_robust(markers, noise, FIELD)
        alarms += bool(dropped)
    assert alarms <= 8                                          # ~2 expected at 99 %


def test_one_moved_marker_is_found():
    """Map says tag 3 is at (1.55, -3.0) but it really stands 1 m further along."""
    rng = np.random.default_rng(2)
    noise = NoiseModel(sigma_pan_deg=1.0)
    found = 0
    for _ in range(50):
        pose = (rng.uniform(-2, 2), rng.uniform(-1.5, 1.5), rng.uniform(-math.pi, math.pi))
        truth = dict(MAP)
        truth[3] = (2.55, -3.0)
        markers = see(pose, {f'm{i}': truth[i] for i in (2, 3, 4, 5)}, rng, noise.sigma_b, 0.05)
        markers = [Landmark(m.label, MAP[int(m.label[1])], m.range, m.bearing, m.side_px)
                   for m in markers]                            # localize with the WRONG map
        fix, dropped = localize_robust(markers, noise, FIELD)
        found += [label for label, _, _ in dropped] == ['m3']
    assert found >= 40
