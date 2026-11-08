"""Independent metric fixtures verify actual edge clearance, not center spacing."""

import math

import pytest

from detection_improved.perception import Detection
from detection_improved.gaps import find_gaps


def obstacle(low_y, high_y, z=1.5, accepted=True):
    low, high = math.atan(low_y / z), math.atan(high_y / z)
    bearing = math.atan((low_y + high_y) / 2 / z)
    return Detection((0, 0, 1, 1), bearing, low, high, z / math.cos(bearing),
                     'depth', 1., 100., high_y - low_y, z, accepted)


@pytest.mark.parametrize('width,count', [(.34, 0), (.35, 1), (.36, 1), (.6, 1)])
def test_35cm_opening_threshold_uses_nearest_edges(width, count):
    gaps = find_gaps([obstacle(-.27, 0), obstacle(width, width + .27)])
    assert len(gaps) == count
    if gaps:
        assert gaps[0].width_m == pytest.approx(width)
        assert gaps[0].center_min <= gaps[0].center_max + 1e-7


def test_center_headings_are_inset_by_half_required_width():
    gap = find_gaps([obstacle(-.6, -.3), obstacle(.3, .6)])[0]
    assert math.tan(gap.center_min) * 1.5 == pytest.approx(-.125)
    assert math.tan(gap.center_max) * 1.5 == pytest.approx(.125)


def test_optional_side_margin_rejects_otherwise_wide_gap():
    assert not find_gaps([obstacle(-.27, 0), obstacle(.36, .63)], margin=.02)


def test_staggered_depth_cannot_create_false_clearance_from_diagonal_distance():
    # Actual facing edges only 20 cm apart laterally, but separated by 2 m in Z.
    assert not find_gaps([obstacle(-.27, 0, z=1.), obstacle(.2, .47, z=3.)])


def test_unmeasured_object_inside_opening_blocks_green_gap():
    bounds = [obstacle(-.7, -.43), obstacle(.43, .7)]
    assert find_gaps(bounds)
    assert not find_gaps(bounds, [{'angle_min': -.02, 'angle_max': .02}])


def test_any_measured_obstacle_bounds_and_splits_gaps_whatever_its_width():
    # Non-target-size (accepted=False) obstacles still bound gaps.
    assert len(find_gaps([obstacle(-.27, 0, accepted=False),
                          obstacle(.36, .63, accepted=False)])) == 1
    # A measured obstacle in the middle splits an 86 cm opening into two 35 cm gaps.
    gaps = find_gaps([obstacle(-.7, -.43), obstacle(-.08, .08, accepted=False),
                      obstacle(.43, .7)])
    assert [g.width_m for g in gaps] == pytest.approx([.35, .35])


def test_no_green_gap_to_unseen_fov_boundary_or_overlapping_objects():
    assert not find_gaps([])
    assert not find_gaps([obstacle(0, .27)])
    assert not find_gaps([obstacle(0, .27), obstacle(.2, .47)])
    assert not find_gaps([obstacle(0, .27), obstacle(.1, .2), obstacle(.4, .67)])


def test_uncertainty_cannot_turn_a_marginal_opening_green():
    right, left = obstacle(-.27, 0), obstacle(.36, .63)
    assert find_gaps([right, left])
    right.width_std_m = left.width_std_m = .01
    assert not find_gaps([right, left])
