"""Independent live fixes, mirror rejection, noisy ranges and rejection gates."""
import math

import numpy as np
import pytest

from l2s2_localisation.live_triangulation import solve
from l2s2_localisation.triangulation import Field, Landmark


def measure(robot=(-3.2, 0), points=((0, -.75), (0, .75)), heading=.3, noise=0, seed=1):
    rng = np.random.default_rng(seed)
    return [Landmark(str(i), tuple(p), math.dist(p, robot)+rng.normal(0, noise),
                     None if heading is None else math.atan2(p[1]-robot[1], p[0]-robot[0])-heading)
            for i, p in enumerate(points)]


@pytest.mark.parametrize('robot', [(-3.2, .2), (3.2, .2)])
def test_two_markers_bearing_order_removes_the_mirror(robot):
    fix = solve(measure(robot))
    assert fix['position_m'][:2] == pytest.approx(robot)
    assert fix['heading_deg'] == pytest.approx(math.degrees(.3))
    assert len(fix['candidates_m']) == 2
    assert 'bearings' in fix['ambiguity_resolution']
    assert np.linalg.eigvalsh(fix['covariance_xy']).min() > 0


def test_unresolved_mirror_is_rejected_instead_of_returning_first_candidate():
    with pytest.raises(ValueError, match='ambiguity'):
        solve(measure(heading=None))
    # Incorrectly identical bearings do not reliably distinguish the branches.
    marks = measure()
    for m in marks:
        m.bearing = 0
    with pytest.raises(ValueError, match='ambiguity'):
        solve(marks)


def test_field_bounds_can_remove_the_extraneous_solution_without_bearings():
    fix = solve(measure((2, .5), [(4.5, -1), (4.5, 1)], heading=None))
    assert fix['position_m'][:2] == pytest.approx((2, .5))
    assert fix['heading_deg'] is None
    assert fix['ambiguity_resolution'] == 'field bounds'


@pytest.mark.parametrize('count', [3, 4, 6])
def test_two_or_more_of_the_six_field_markers(count):
    points = [(-.7, 0), (-1.5, -3), (-1.5, 3), (1.55, -3), (1.55, 3), (4.5, 0)][:count]
    fix = solve(measure((-2.8, .4), points, heading=-.7))
    assert fix['position_m'][:2] == pytest.approx((-2.8, .4), abs=1e-6)
    assert len(fix['used_ids']) == count


def test_noisy_ranges_use_all_markers_and_remain_close_to_truth():
    points = [(-.7, 0), (-1.5, -3), (-1.5, 3), (1.55, -3), (1.55, 3), (4.5, 0)]
    fix = solve(measure((-2.8, .4), points, noise=.04))
    assert math.dist(fix['position_m'][:2], (-2.8, .4)) < .1
    assert fix['rms_range_error_m'] < .1


def test_collinear_markers_still_use_bearings_to_choose_a_side():
    fix = solve(measure((-2, .4), [(0, -2), (0, 0), (0, 2)]))
    assert fix['position_m'][:2] == pytest.approx((-2, .4))
    assert 'bearings' in fix['ambiguity_resolution']


@pytest.mark.parametrize('case', ['one', 'duplicate', 'baseline', 'nonfinite', 'bad_ranges', 'poor_geometry', 'outside'])
def test_invalid_or_untrustworthy_input_is_rejected(case):
    marks = measure()
    if case == 'one':
        marks = marks[:1]
    elif case == 'duplicate':
        marks[1].label = marks[0].label
    elif case == 'baseline':
        marks[1].position = (0, -.74)
    elif case == 'nonfinite':
        marks[0].range = float('nan')
    elif case == 'bad_ranges':
        marks = measure((-2, .4), [(0, -2), (0, 2), (3, 0)])
        marks[0].range += 10
    elif case == 'poor_geometry':
        marks = measure((0, 0), [(-1, 0), (1, 0)], heading=None)
    elif case == 'outside':
        marks = measure((10, 0))
    with pytest.raises(ValueError):
        solve(marks)
