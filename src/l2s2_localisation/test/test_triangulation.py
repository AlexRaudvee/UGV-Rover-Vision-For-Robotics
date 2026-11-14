"""Triangulation tests (pure Python)."""

import math

import numpy as np
import pytest

from l2s2_localisation.localize import landmarks_from_scan, load_map, localize_scan
from l2s2_localisation.triangulation import (Field, Landmark, circle_intersections,
                                               localize, radical_line)


def landmarks(robot, markers, heading=None, noise=0.0, seed=0):
    """Exact (or noisy) ranges and bearings from a robot pose to markers."""
    rng = np.random.default_rng(seed)
    result = []
    for index, (x, y) in enumerate(markers):
        distance = math.hypot(x - robot[0], y - robot[1]) + rng.normal(0, noise) * bool(noise)
        bearing = (None if heading is None else
                   math.atan2(y - robot[1], x - robot[0]) - heading)
        result.append(Landmark(f'm{index}', (x, y), distance, bearing))
    return result


def contains_point(candidates, point, tol=1e-9):
    return any(np.allclose(c, point, atol=tol) for c in candidates)


def test_radical_line_matches_hand_derivation():
    a, b, c = radical_line((1.0, 2.0), 3.0, (4.0, -1.0), 2.0)
    assert (a, b) == (6.0, -6.0)
    assert c == pytest.approx(9 - 4 + (16 + 1) - (1 + 4))


def test_worked_example_two_solutions_and_field_check():
    """Markers on the goal line; the mirror solution lies beyond the field."""
    p1, p2, robot = (4.5, 1.0), (4.5, -1.0), (2.0, 0.5)
    marks = landmarks(robot, [p1, p2])
    candidates, meet = circle_intersections(p1, marks[0].range, p2, marks[1].range)
    assert meet
    assert contains_point(candidates, (2.0, 0.5)) and contains_point(candidates, (7.0, 0.5))
    fix = localize(marks)
    assert np.allclose(fix.position, robot)
    assert 'field' in fix.method


def test_b_zero_markers_at_same_y():
    """B = 0 (y1 = y2): the x = C / A branch of the derivation."""
    p1, p2, robot = (-2.0, 3.5), (2.0, 3.5), (0.5, 1.0)
    marks = landmarks(robot, [p1, p2])
    candidates, _ = circle_intersections(p1, marks[0].range, p2, marks[1].range)
    assert contains_point(candidates, robot) and contains_point(candidates, (0.5, 6.0))


def test_tiny_b_is_numerically_stable():
    p1, p2, robot = (-2.0, 3.5), (2.0, 3.5 + 1e-12), (0.5, 1.0)
    marks = landmarks(robot, [p1, p2])
    candidates, _ = circle_intersections(p1, marks[0].range, p2, marks[1].range)
    assert contains_point(candidates, robot, tol=1e-6)


@pytest.mark.parametrize('seed', range(20))
def test_random_configurations_recover_robot(seed):
    rng = np.random.default_rng(seed)
    p1, p2, robot = (tuple(rng.uniform(-5, 5, 2)) for _ in range(3))
    marks = landmarks(robot, [p1, p2])
    candidates, meet = circle_intersections(p1, marks[0].range, p2, marks[1].range)
    assert meet and contains_point(candidates, robot, tol=1e-6)


def test_circles_that_miss_give_closest_point_and_warning():
    marks = [Landmark('a', (0.0, 0.0), 1.0), Landmark('b', (3.0, 0.0), 1.5)]   # 1 + 1.5 < 3
    fix = localize(marks)
    assert fix.position[1] == pytest.approx(0.0)
    assert 1.0 < fix.position[0] < 1.5
    assert any('do not intersect' in w for w in fix.warnings)


def test_bearings_pick_the_right_solution_inside_the_field():
    """Both solutions are on the field; only the bearings tell them apart."""
    markers = [(0.0, 1.0), (0.0, -1.0)]
    for robot in [(-1.0, 0.3), (1.0, 0.3)]:            # the two mirror positions
        fix = localize(landmarks(robot, markers, heading=0.4))
        assert np.allclose(fix.position, robot, atol=1e-9)
        assert fix.heading == pytest.approx(0.4)
        assert 'bearing' in fix.method


def test_without_bearings_the_ambiguity_is_reported():
    fix = localize(landmarks((-1.0, 0.3), [(0.0, 1.0), (0.0, -1.0)]))
    assert any('two solutions' in w for w in fix.warnings)


def test_three_markers_exact():
    markers = [(0.0, 3.9), (0.0, -3.9), (-5.4, 0.0)]     # like the hand sketch
    robot = (-1.2, 0.8)
    fix = localize(landmarks(robot, markers, heading=-0.7))
    assert np.allclose(fix.position, robot, atol=1e-9)
    assert fix.heading == pytest.approx(-0.7)
    assert 'least squares' in fix.method


def test_refinement_beats_linear_least_squares_with_noise():
    markers = [(0.0, 3.9), (0.0, -3.9), (-5.4, 0.0), (5.4, 1.0)]
    robot = np.array([1.5, -0.5])
    linear_errors, refined_errors = [], []
    for seed in range(30):
        marks = landmarks(robot, markers, noise=0.05, seed=seed)
        linear_errors.append(np.linalg.norm(localize(marks, use_refine=False).position - robot))
        refined_errors.append(np.linalg.norm(localize(marks).position - robot))
    assert np.mean(refined_errors) <= np.mean(linear_errors)
    assert np.mean(refined_errors) < 0.1


def test_collinear_markers_fall_back_to_two_circles():
    markers = [(4.5, 1.0), (4.5, 0.0), (4.5, -1.0)]       # all on the goal line
    robot = (2.0, 0.5)
    fix = localize(landmarks(robot, markers))
    assert np.allclose(fix.position, robot, atol=1e-6)
    assert any('collinear' in w for w in fix.warnings)


def test_localize_scan_with_map(tmp_path):
    path = tmp_path / 'map.yaml'
    path.write_text('markers:\n  tag36h11:1: [0.0, 3.9]\n  tag36h11:2: [0.0, -3.9]\n'
                    '  tag36h11:3: [-5.4, 0.0]\n')
    world, field_model = load_map(path)
    assert field_model == Field()
    robot, heading = (-1.2, 0.8), 0.3
    scan = []
    for (family, marker_id), (x, y) in world.items():
        bearing = math.degrees(math.atan2(y - robot[1], x - robot[0]) - heading)
        scan.append({'family': family, 'id': marker_id,
                     'range_m': math.hypot(x - robot[0], y - robot[1]),
                     'bearing_deg': (bearing + 180) % 360 - 180})
    scan.append({'family': '6X6_250', 'id': 9, 'range_m': 1.0, 'bearing_deg': 0.0})
    result = localize_scan(scan, world, field_model)
    assert result['x_m'] == pytest.approx(-1.2, abs=1e-3)
    assert result['y_m'] == pytest.approx(0.8, abs=1e-3)
    assert result['heading_deg'] == pytest.approx(math.degrees(heading), abs=0.1)
    assert result['markers_not_in_map'] == ['6X6_250:9']
    marks, _ = landmarks_from_scan(scan[:1], world)
    assert 'error' in localize_scan(scan[:1], world, field_model) and len(marks) == 1
