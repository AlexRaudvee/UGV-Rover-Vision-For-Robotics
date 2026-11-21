"""Path planner tests (no ROS)."""
import math

import pytest

from l2s2_localisation.path_plan import TARGETS, PathPlanner, along_and_offset, line_points


def test_targets_match_assignment_table():
    assert TARGETS[1][1:] == (3.20, 0.00)
    assert TARGETS[2][1:] == (3.90, 1.10)
    assert TARGETS[5][1:] == (2.85, -2.00)


def test_line_points_start_and_end_on_the_ends():
    points = line_points((-3.2, 0.0), (3.2, 0.0), spacing=0.1)
    assert points[0] == pytest.approx((-3.2, 0.0)) and points[-1] == pytest.approx((3.2, 0.0))
    assert all(math.dist(a, b) <= 0.1 + 1e-9 for a, b in zip(points, points[1:]))


def test_offset_is_positive_on_the_left():
    along, offset = along_and_offset((1.0, 0.5), (0.0, 0.0), (4.0, 0.0))
    assert along == pytest.approx(1.0) and offset == pytest.approx(0.5)
    assert along_and_offset((1.0, -0.5), (0.0, 0.0), (4.0, 0.0))[1] == pytest.approx(-0.5)


def test_first_update_plans_from_the_current_position():
    planner = PathPlanner(TARGETS[1][1:])
    status, replanned = planner.update((-3.2, 0.0, 0.0))
    assert replanned and status['plan_reason'] == 'first plan'
    assert status['distance_m'] == pytest.approx(6.4)
    assert status['heading_error_deg'] == pytest.approx(0.0)


def test_small_noise_does_not_replan():
    planner = PathPlanner((3.2, 0.0))
    planner.update((-3.2, 0.0, 0.0))
    for k in range(1, 30):                       # drive along with 5 cm sideways wobble
        _, replanned = planner.update((-3.2 + 0.1 * k, 0.05 * math.sin(k), 0.0))
        assert not replanned
    assert planner.count == 1


def test_jump_in_position_replans_from_the_new_position():
    planner = PathPlanner((3.2, 0.0))
    planner.update((-3.2, 0.0, 0.0))
    planner.update((-3.0, 0.0, 0.0))
    status, replanned = planner.update((-2.95, 0.45, 0.0))    # a marker moved the estimate
    assert replanned and status['plan_reason'].startswith('position jumped')
    assert status['plan_start'] == pytest.approx([-2.95, 0.45])
    assert status['offset_m'] == pytest.approx(0.0)


def test_drifting_off_the_line_replans():
    planner = PathPlanner((3.2, 0.0), max_offset=0.3)
    planner.update((-3.2, 0.0, 0.0))
    replans = []
    for k in range(1, 40):                       # slowly drift left, 2 cm per step
        _, replanned = planner.update((-3.2 + 0.1 * k, 0.02 * k, 0.0))
        replans.append(replanned)
    assert any(replans)
    assert 'off the line' in planner.plan.reason


def test_new_target_replans():
    planner = PathPlanner((3.2, 0.0))
    planner.update((0.0, 0.0, 0.0))
    planner.set_goal((2.85, 2.0))
    status, replanned = planner.update((0.0, 0.0, 0.0))
    assert replanned and status['plan_reason'] == 'new target'
    assert status['heading_error_deg'] == pytest.approx(math.degrees(math.atan2(2.0, 2.85)), abs=0.1)


def test_arrival_and_no_replan_near_the_target():
    planner = PathPlanner((3.2, 0.0), arrive=0.15)
    planner.update((3.0, 0.0, 0.0))
    status, replanned = planner.update((3.1, 0.6, 0.0))      # big jump, but close: still re-plans
    status, replanned = planner.update((3.15, 0.05, 0.0))
    assert status['arrived'] and not replanned
