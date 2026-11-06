"""Test the state machine on a simulated robot, using the real gap finder."""

import json
import math
import random

from object_detection.perception import Detection
from obstacle_avoidance.controller import (
    AvoidanceController, choose_plan, CRUISE, DONE, ESCAPE, front_blocked, HOLD, lane_clear,
    least_blocked_side, Obstacle, parse_detections, parse_gaps, PASS, path_blocked, Perception, RETURN, ROTATE,
    Settings, space_blocked, TURN,
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


def simulate(boxes, seconds=90.0, noise=None, hidden=(), **overrides):
    """Drive a simulated robot and record what happened. Only the LiDAR sees hidden boxes."""
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
        points = scan_points(pose, list(boxes) + list(hidden))
        command = controller.step(now, tuple(pose), perception, front_blocked(points, settings),
                                  space_blocked(points, settings), points)
        pose[0] += command.linear * math.cos(pose[2]) * 0.1
        pose[1] += command.linear * math.sin(pose[2]) * 0.1
        pose[2] += command.angular * 0.1
        if boxes or hidden:
            clearance = min(clearance, min(distance_to_box(pose[0], pose[1], box)
                                           for box in list(boxes) + list(hidden)))
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
    for state in (RETURN, PASS, TURN, HOLD, ROTATE, ESCAPE):
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


def test_stays_safe_and_near_the_path_when_a_wall_blocks_everything():
    wall = [(1.3, y * 0.3) for y in range(-6, 7)]
    result = simulate(wall, seconds=130.0, target_distance=3.0)
    assert result['clearance'] > ROBOT_RADIUS
    assert ROTATE in result['states']
    assert result['y_max'] < 1.2
    assert result['note'] != 'reached the target distance'


def test_rotates_around_an_obstacle_that_only_the_lidar_sees():
    result = simulate([], hidden=[(1.3, 0.0)], target_distance=3.0)
    assert_finished_on_path(result)
    assert ROTATE in result['states'] and ESCAPE in result['states']


def test_follows_the_path_direction_past_the_obstacle_after_escaping():
    result = simulate([], hidden=[(1.3, 0.0)], target_distance=3.0, stop_half_width=0.20)
    assert_finished_on_path(result)
    states = result['states']
    assert states[states.index(ESCAPE) + 1] == PASS
    assert result['y_max'] < 0.5


def test_short_block_only_pauses_and_never_rotates():
    controller = AvoidanceController(Settings(blocked_timeout=5.0))
    pose = (0.0, 0.0, 0.0)
    assert controller.step(0.0, pose, Perception(0.0), False).linear > 0
    for tick in range(1, 30):
        command = controller.step(tick * 0.1, pose, Perception(tick * 0.1), True, True)
        assert command.linear == 0 and command.angular == 0
        assert command.state == CRUISE and 'LiDAR stop' in command.note
    assert controller.step(3.0, pose, Perception(3.0), False).linear > 0


@pytest.mark.parametrize('side,sign', [('left', 1), ('right', -1)])
def test_rotates_after_the_timeout_towards_the_preferred_side(side, sign):
    controller = AvoidanceController(Settings(preferred_side=side, blocked_timeout=1.0))
    pose = (0.0, 0.0, 0.0)
    assert controller.step(0.0, pose, Perception(0.0), False).linear > 0
    assert controller.step(0.1, pose, Perception(0.1), True, True).state == CRUISE
    command = controller.step(1.5, pose, Perception(1.5), True, True)
    assert command.state == ROTATE
    assert command.linear == 0 and command.angular * sign > 0


def rotate_until_space(world_points):
    """Block a controller until it rotates, then free the way ahead and return it."""
    controller = AvoidanceController(Settings(blocked_timeout=1.0))
    controller.step(0.0, (0.0, 0.0, 0.0), Perception(0.0), False, False, [])
    controller.step(0.1, (0.0, 0.0, 0.0), Perception(0.1), True, True, [])
    command = controller.step(1.5, (0.0, 0.0, 0.0), Perception(1.5), True, True, [])
    assert command.state == ROTATE
    yaw = 0.0
    for tick in range(16, 60):
        yaw += command.angular * 0.1
        points = [to_body((0.0, 0.0, yaw), point) for point in world_points]
        command = controller.step(tick * 0.1, (0.0, 0.0, yaw), Perception(tick * 0.1),
                                  False, False, points)
        if controller.state != ROTATE:
            break
    return controller


def test_escape_runs_only_if_the_path_is_still_blocked_after_rotating():
    assert rotate_until_space([(0.3, 0.0), (0.3, 0.05), (0.3, -0.05)]).state == ESCAPE


def test_no_escape_when_the_path_is_free_after_rotating():
    assert rotate_until_space([]).state == CRUISE


def test_path_check_uses_the_path_direction_not_the_robot_heading():
    settings = Settings()
    box = [(0.3, 0.0), (0.3, 0.05)]
    assert path_blocked(box, 0.0, settings)
    assert path_blocked([to_body((0.0, 0.0, 1.0), p) for p in box], 1.0, settings)
    assert not path_blocked(box, 1.0, settings)
    # Turned far enough that the box is almost beside the robot, but still along the path.
    assert path_blocked([to_body((0.0, 0.0, 1.4), p) for p in box], 1.4, settings)


def box_points(center_y, x=0.35):
    """Sample the front of a 30 cm box as the LiDAR would see it."""
    return [(x, center_y - BOX / 2 + step * BOX / 12) for step in range(13)]


@pytest.mark.parametrize('center_y,expected', [(-0.05, 1), (-0.15, 1), (0.05, -1), (0.15, -1),
                                               (0.0, 0)])
def test_least_blocked_side(center_y, expected):
    assert least_blocked_side(box_points(center_y), Settings()) == expected


def test_a_second_obstacle_makes_that_side_more_blocked():
    # The nearest box is centered, but another one is on the left, so right is freer.
    points = box_points(0.0) + box_points(0.45)
    assert least_blocked_side(points, Settings()) == -1


def rotation_sign(center_y, **overrides):
    """Return the direction of the first rotation with a box in front of the robot."""
    controller = AvoidanceController(Settings(blocked_timeout=1.0, **overrides))
    pose = (0.0, 0.0, 0.0)
    points = box_points(center_y)
    controller.step(0.0, pose, Perception(0.0), False, False, [])
    controller.step(0.1, pose, Perception(0.1), True, True, points)
    command = controller.step(1.5, pose, Perception(1.5), True, True, points)
    assert command.state == ROTATE
    return math.copysign(1, command.angular)


@pytest.mark.parametrize('center_y,sign', [(-0.05, 1), (0.05, -1)])
def test_rotates_away_from_the_obstruction(center_y, sign):
    assert rotation_sign(center_y) == sign


def test_least_obstruction_wins_over_the_preferred_side():
    assert rotation_sign(-0.1, preferred_side='right') == 1
    assert rotation_sign(0.1, preferred_side='left') == -1


@pytest.mark.parametrize('side,sign', [('left', 1), ('right', -1)])
def test_preferred_side_decides_when_both_sides_are_even(side, sign):
    assert rotation_sign(0.0, preferred_side=side) == sign


def test_camera_decides_when_the_lidar_has_no_points():
    controller = AvoidanceController(Settings(blocked_timeout=1.0))
    pose = (0.0, 0.0, 0.0)
    # Off to the right and outside the path, so it does not start normal avoidance.
    seen = [Obstacle(-0.5, -0.6, -0.4, 0.9)]
    controller.step(0.0, pose, Perception(0.0), False, False, [])
    controller.step(0.1, pose, Perception(0.1, seen), True, True, [])
    command = controller.step(1.5, pose, Perception(1.5, seen), True, True, [])
    assert command.state == ROTATE and command.angular > 0


def test_no_safe_heading_waits_in_hold_before_rotating():
    controller = AvoidanceController(Settings(blocked_timeout=2.0))
    pose = (0.0, 0.0, 0.0)
    wall = [Obstacle(0.0, -1.2, 1.2, 0.8)]
    for now in (0.0, 1.0, 1.9):
        command = controller.step(now, pose, Perception(now, wall), False, False, [])
        assert command.state == HOLD and command.linear == 0 and command.angular == 0
    command = controller.step(2.5, pose, Perception(2.5, wall), False, False, [])
    assert command.state == ROTATE and command.angular != 0


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


def test_rotation_does_not_end_until_there_is_space():
    controller = AvoidanceController(Settings(blocked_timeout=0.0))
    yaw, command = 0.0, None
    controller.step(-0.2, (0.0, 0.0, yaw), Perception(-0.2), False)
    controller.step(-0.1, (0.0, 0.0, yaw), Perception(-0.1), True, True)
    for tick in range(20):
        command = controller.step(tick * 0.1, (0.0, 0.0, yaw), Perception(tick * 0.1), True, True)
        assert command.state == ROTATE and command.linear == 0
        yaw += command.angular * 0.1
    for tick in range(20, 40):
        command = controller.step(tick * 0.1, (0.0, 0.0, yaw), Perception(tick * 0.1), False, False)
        yaw += command.angular * 0.1
        if controller.state == ESCAPE:
            break
    assert controller.state == ESCAPE
    assert controller.step(4.0, (0.0, 0.0, yaw), Perception(4.0), False, False).linear > 0


def test_gives_up_after_looking_to_both_sides():
    controller = AvoidanceController(Settings(preferred_side='left', blocked_timeout=0.0))
    yaw, signs = 0.0, set()
    controller.step(-0.2, (0.0, 0.0, yaw), Perception(-0.2), False)
    controller.step(-0.1, (0.0, 0.0, yaw), Perception(-0.1), True, True)
    for tick in range(400):
        command = controller.step(tick * 0.1, (0.0, 0.0, yaw), Perception(tick * 0.1), True, True)
        assert command.linear == 0
        if command.angular:
            signs.add(math.copysign(1, command.angular))
        yaw += command.angular * 0.1
        if controller.state == DONE:
            break
    assert controller.note == 'no space found on either side'
    assert signs == {1.0, -1.0}
    assert abs(yaw) <= 1.57 + 0.05


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
