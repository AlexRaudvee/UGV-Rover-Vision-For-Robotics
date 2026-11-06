"""Test the state machine on a simulated robot, using the real gap finder."""

import json
import math
import random

from object_detection.perception import Detection
from obstacle_avoidance.controller import (
    AvoidanceController, choose_plan, CRUISE, DONE, front_blocked, HOLD, lane_clear, Obstacle,
    parse_detections, parse_gaps, PASS, Perception, RETURN, Settings, TURN,
)
from polar_hist.gaps import find_gaps
import pytest

FOV = 0.6
ROBOT_RADIUS = 0.175
BOX = 0.30


def to_body(pose, point):
    """Convert a world point to the robot's frame."""
    dx, dy = point[0] - pose[0], point[1] - pose[1]
    return (dx * math.cos(pose[2]) + dy * math.sin(pose[2]),
            -dx * math.sin(pose[2]) + dy * math.cos(pose[2]))


def see(pose, boxes, settings, noise=None):
    """Build the messages that the detector and polar_hist would publish."""
    blockers, unknown = [], []
    noise = noise or (lambda: 0.0)
    for bx, by in boxes:
        corners = [to_body(pose, (bx + i * BOX / 2, by + j * BOX / 2))
                   for i in (-1, 1) for j in (-1, 1)]
        corners = [(x - settings.camera_forward, y) for x, y in corners]
        if min(x for x, _ in corners) <= 0.05:
            continue
        angles = [math.atan2(y, x) for x, y in corners]
        low, high = min(angles) + noise(), max(angles) + noise()
        if high < -FOV or low > FOV:
            continue
        z = min(x for x, _ in corners)
        # A close box touches the bottom of the image, and so has no measurement.
        if low <= -FOV or high >= FOV or z < 0.45:
            unknown.append({'angle_min': max(low, -FOV), 'angle_max': min(high, FOV)})
            continue
        bearing = (low + high) / 2
        blockers.append(Detection((0, 0, 1, 1), bearing, low, high, z / math.cos(bearing),
                                  'lidar', 1.0, 100.0, BOX, z))
    gaps = find_gaps(blockers, unknown, settings.robot_width)
    detections = {'schema_version': 1, 'fresh': True,
                  'blockers': [item.to_dict() for item in blockers],
                  'unknown_intervals': unknown}
    gap_message = {'schema_version': 1, 'fresh': True,
                   'gaps': [item.to_dict() for item in gaps]}
    return json.loads(json.dumps(detections)), json.loads(json.dumps(gap_message))


def scan_points(pose, boxes):
    """Sample the box outlines, as the LiDAR would see them."""
    points = []
    for bx, by in boxes:
        for step in range(13):
            offset = -BOX / 2 + step * BOX / 12
            for world in ((bx + offset, by - BOX / 2), (bx + offset, by + BOX / 2),
                          (bx - BOX / 2, by + offset), (bx + BOX / 2, by + offset)):
                points.append(to_body(pose, world))
    return points


def distance_to_box(x, y, box):
    """Return the distance from a point to a box's outline."""
    dx = max(abs(x - box[0]) - BOX / 2, 0.0)
    dy = max(abs(y - box[1]) - BOX / 2, 0.0)
    return math.hypot(dx, dy)


def simulate(boxes, seconds=90.0, noise=None, **overrides):
    """Drive a simulated robot and record what happened."""
    settings = Settings(**overrides)
    controller = AvoidanceController(settings)
    pose = [0.0, 0.0, 0.0]
    perception, clearance, y_max = None, math.inf, 0.0
    states, sides = [], set()
    for tick in range(int(seconds * 10)):
        now = tick * 0.1
        if tick % 2 == 0:  # Detections arrive at 5 Hz.
            detections, gaps = see(pose, boxes, settings, noise)
            perception = parse_detections(detections, now)
            perception.gaps = parse_gaps(gaps)
        blocked = front_blocked(scan_points(pose, boxes), settings)
        command = controller.step(now, tuple(pose), perception, blocked)
        pose[0] += command.linear * math.cos(pose[2]) * 0.1
        pose[1] += command.linear * math.sin(pose[2]) * 0.1
        pose[2] += command.angular * 0.1
        if boxes:
            clearance = min(clearance, min(distance_to_box(pose[0], pose[1], box)
                                           for box in boxes))
        y_max = max(y_max, abs(pose[1]))
        if not states or states[-1] != command.state:
            states.append(command.state)
        if controller.plan is not None:
            sides.add(controller.plan.side)
        if controller.state == DONE:
            break
    return {'pose': pose, 'clearance': clearance, 'states': states, 'y_max': y_max,
            'sides': sides, 'note': controller.note}


def assert_finished_on_path(result):
    assert result['note'] == 'reached the target distance'
    assert result['clearance'] > ROBOT_RADIUS
    assert abs(result['pose'][1]) < 0.1


def test_clear_path_drives_straight_to_the_target():
    result = simulate([])
    assert result['states'] == [CRUISE, DONE]
    assert result['y_max'] < 0.01


@pytest.mark.parametrize('side', ['left', 'right'])
def test_passes_a_single_obstacle_on_either_side(side):
    result = simulate([(1.3, 0.0)], target_distance=3.0, preferred_side=side)
    assert_finished_on_path(result)
    assert result['sides'] == {side}
    assert (result['y_max'] > 0.2)
    assert 'RETURN' in result['states']


@pytest.mark.parametrize('offset', [-0.25, -0.1, 0.1, 0.25])
def test_passes_on_the_shorter_side_of_an_offset_obstacle(offset):
    result = simulate([(1.3, offset)], target_distance=3.0)
    assert_finished_on_path(result)
    assert result['sides'] == {'right' if offset > 0 else 'left'}


def test_ignores_an_obstacle_outside_the_path():
    result = simulate([(1.3, 0.8)], target_distance=3.0)
    assert_finished_on_path(result)
    assert result['y_max'] < 0.01


def test_drives_through_a_gap_between_two_obstacles():
    result = simulate([(1.3, 0.5), (1.3, -0.5)], target_distance=3.0)
    assert_finished_on_path(result)
    assert result['y_max'] < 0.15


def test_picks_the_open_side_when_one_side_is_blocked():
    result = simulate([(1.3, 0.0), (1.3, 0.35)], target_distance=3.0)
    assert_finished_on_path(result)
    assert result['sides'] == {'right'}


def test_passes_two_staggered_obstacles_in_a_row():
    result = simulate([(1.3, 0.0), (3.0, 0.1)], target_distance=4.5)
    assert_finished_on_path(result)


def test_does_not_cut_across_in_front_of_a_second_obstacle():
    # The first pass ends on the right, so the second one should stay on the right.
    result = simulate([(1.3, 0.0), (3.0, 0.1)], target_distance=4.5, preferred_side='right')
    assert_finished_on_path(result)
    assert result['sides'] == {'right'}


def test_plan_rejects_a_lane_blocked_behind_the_first_obstacle():
    # The angled drive to the right misses the second box, but the lane after it does not.
    settings = Settings(preferred_side='right')
    pose = (0.0, 0.0, 0.0)
    near = [Obstacle(0.0, -0.15, 0.15, 1.0)]
    other = Obstacle(math.atan(-0.14), math.atan(-0.21), math.atan(-0.07), 1.444)
    seen = Perception(0.0, near + [other])
    closest_right_lane = -1.0 * math.tan(0.15) - settings.clearance
    plan = choose_plan(pose, seen, near, settings)
    assert not lane_clear(closest_right_lane, pose, plan.clear_x, seen, settings)
    assert lane_clear(plan.lane_y, pose, plan.clear_x, seen, settings)


def test_target_distance_stops_the_robot_in_every_state():
    for state in (RETURN, PASS, TURN, HOLD):
        controller = AvoidanceController(Settings(target_distance=1.0))
        controller.step(0.0, (0.0, 0.0, 0.0), Perception(0.0), False)
        controller.state = state
        command = controller.step(0.1, (1.1, 0.3, 0.0), Perception(0.1), False)
        assert command.state == DONE and command.linear == 0


def test_side_stays_fixed_with_noisy_measurements():
    rng = random.Random(4)
    result = simulate([(1.3, 0.0)], target_distance=3.0, noise=lambda: rng.gauss(0, 0.01))
    assert_finished_on_path(result)
    assert len(result['sides']) == 1


def test_gives_up_safely_when_a_wall_blocks_everything():
    wall = [(1.3, y * 0.3) for y in range(-6, 7)]
    result = simulate(wall, target_distance=3.0, blocked_timeout=5.0)
    assert result['clearance'] > ROBOT_RADIUS
    assert HOLD in result['states'] or result['states'][-1] == DONE
    assert result['note'] != 'reached the target distance'


@pytest.mark.parametrize('pose,perception,scan,reason', [
    (None, Perception(0.0), False, 'odometry'),
    ((0.0, 0.0, 0.0), None, False, 'detections'),
    ((0.0, 0.0, 0.0), Perception(-5.0), False, 'detections'),
    ((0.0, 0.0, 0.0), Perception(0.0), None, 'LiDAR'),
])
def test_missing_or_old_data_stops_the_robot(pose, perception, scan, reason):
    command = AvoidanceController().step(0.0, pose, perception, scan)
    assert command.linear == 0 and command.angular == 0
    assert reason in command.note


def test_lidar_stop_overrides_driving_and_then_times_out():
    controller = AvoidanceController(Settings(blocked_timeout=1.0))
    pose = (0.0, 0.0, 0.0)
    assert controller.step(0.0, pose, Perception(0.0), False).linear > 0
    assert controller.step(0.1, pose, Perception(0.1), True).linear == 0
    assert controller.step(0.2, pose, Perception(0.2), False).linear > 0
    assert controller.step(0.3, pose, Perception(0.3), True).linear == 0
    assert controller.step(1.5, pose, Perception(1.5), True).state == DONE


def test_message_that_is_not_fresh_is_rejected():
    assert parse_detections({'schema_version': 1, 'fresh': False}, 0.0) is None
    assert parse_gaps({'schema_version': 1, 'fresh': False}) == []


def test_start_pose_defines_the_path_when_the_robot_starts_rotated():
    controller = AvoidanceController()
    controller.step(0.0, (1.0, 2.0, math.pi / 2), Perception(0.0), False)
    x, y, yaw = controller.to_path_frame((1.0, 3.0, math.pi / 2))
    assert x == pytest.approx(1.0)
    assert y == pytest.approx(0.0)
    assert yaw == pytest.approx(0.0)
