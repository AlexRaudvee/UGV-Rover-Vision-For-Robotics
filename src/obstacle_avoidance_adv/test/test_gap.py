"""Gap finding between two obstacles; pure planner, no ROS."""
import math

import numpy as np
import pytest

from obstacle_avoidance_adv.planner import Controller, Settings, View, rotation


class SeenEverywhere:
    """A scan that has observed every direction and has no points of its own."""
    points = np.empty((0, 2))

    def observed(self, point):
        return True


def box(x, y, size=.30):
    """Outline of a square box centred at (x, y), one point per centimetre."""
    h = size / 2
    edge = np.linspace(-h, h, int(size * 100) + 1)
    return np.vstack([np.column_stack((x + edge, np.full_like(edge, y - h))),
                      np.column_stack((x + edge, np.full_like(edge, y + h))),
                      np.column_stack((np.full_like(edge, x - h), y + edge)),
                      np.column_stack((np.full_like(edge, x + h), y + edge))])


def view(world, pose=(0., 0., 0.), cfg=None):
    return View([SeenEverywhere()], np.array(pose), world, cfg or Settings(goal_distance=5.), 0.)


def started(cfg):
    c = Controller(cfg)
    c.start(np.zeros(3), 0.)
    return c


def test_gap_between_two_boxes_is_found_on_the_correct_side():
    # Box on the route, second box to the left: the opening is 0.55 m wide, centred 0.425 m left.
    world = np.vstack([box(1.7, 0.), box(1.7, .85)])
    plan = view(world).gap_plan(0.)
    assert plan is not None
    heading, distance, through = plan
    entry = np.array([math.cos(heading), math.sin(heading)]) * distance
    assert heading > 0                                   # left
    assert abs(entry[1] - .425) < .03                    # in line with the middle of the opening
    assert entry[0] < 1.55 - .13 - .30                   # stops short of the boxes for the run-up
    assert entry[0] + through > 1.85 + .13               # drives fully past the boxes


def test_gap_on_the_right_is_found_on_the_right():
    world = np.vstack([box(1.7, 0.), box(1.7, -.85)])
    heading, _, _ = view(world).gap_plan(0.)
    assert heading < 0


def test_staggered_boxes_still_form_a_gap():
    # Second box 15 cm closer than the box on the route, as placed on the real field.
    world = np.vstack([box(1.7, 0.), box(1.55, .85)])
    heading, _, _ = view(world).gap_plan(0.)
    assert heading > 0


def test_opening_measured_centre_to_centre_is_too_narrow():
    # Centres 0.62 m apart leave a 0.32 m opening, as seen by the LiDAR on the rover.
    world = np.vstack([box(1.8, .22), box(1.7, -.40)])
    assert view(world).gap_plan(0.) is None


def test_rover_layout_with_object_beside_the_rover():
    # Scan from the rover: boxes at y -0.29..+0.04 and +0.67..+0.99, about 1.24 m ahead,
    # and a straight object beside the rover (x 0.02..0.32, y +0.37) that blocks the
    # drive to the middle of the opening but not to its right-hand part.
    beside = np.column_stack((np.linspace(.02, .32, 31), np.full(31, .375)))
    world = np.vstack([box(1.39, -.125), box(1.37, .83), beside])
    v = view(world, cfg=Settings(goal_distance=5., half_width=.11, half_length=.125))   # the 22 x 25 cm rover
    plan = v.gap_plan(0.)
    assert plan is not None
    heading, distance, _ = plan
    y = distance * math.sin(heading)
    assert .2 < y < .32
    assert v.gap_reachable(0., distance * math.cos(heading), 1., y)


def test_single_box_is_not_a_gap():
    assert view(box(1.8, 0.)).gap_plan(0.) is None


def test_gap_narrower_than_the_corridor_is_rejected():
    # Opening 0.40 m; the corridor needs 2 * (0.15 + 0.08) = 0.46 m.
    world = np.vstack([box(1.8, 0.), box(1.8, .70)])
    assert view(world).gap_plan(0.) is None


def test_gap_blocked_behind_is_rejected():
    # A third box right behind the opening closes it.
    world = np.vstack([box(1.8, 0.), box(1.8, .85), box(2.35, .425)])
    assert view(world).gap_plan(0.) is None


def test_controller_enters_gap_instead_of_stopping():
    cfg = Settings(goal_distance=5.)
    c = started(cfg)
    world = np.vstack([box(1.8, 0.), box(1.8, .85)])
    pose = np.array([.1, 0., 0.])                      # obstacle now inside the 1.5 m lookahead
    c.step(pose, view(world, pose), 0.1)
    assert c.state == 'GAP'
    assert c.gap_heading > 0


def test_single_box_still_uses_the_quarter_turn_sidestep():
    cfg = Settings(goal_distance=5.)
    pose = np.array([1.8 - .15 - cfg.half_length - cfg.stop_clearance + .01, 0., 0.])
    c = started(cfg)
    world = box(1.8, 0.)
    c.step(pose, view(world, pose), 0.1)
    assert c.state == 'STOP'
    for t in (0.2, 0.5, 0.8):
        c.step(pose, view(world, pose), t)
    assert c.state == 'TURN_OUT'


def simulate(world, cfg, seconds=120., dt=.05):
    """Drive the controller with ideal motion and return the states, the path and the closest approach."""
    c = started(cfg)
    pose = np.zeros(3)
    states, path = [], []
    closest = math.inf
    for i in range(int(seconds / dt)):
        now = i * dt
        v, w = c.step(pose, view(world, pose, cfg), now, w if i else 0., v if i else 0.)
        pose = pose + np.array([v * math.cos(pose[2]) * dt, v * math.sin(pose[2]) * dt, w * dt])
        local = (world - pose[:2]) @ rotation(pose[2])
        inside = np.maximum(abs(local[:, 0]) - cfg.half_length, abs(local[:, 1]) - cfg.half_width)
        closest = min(closest, float(inside.min()))
        if not states or states[-1] != c.state:
            states.append(c.state)
        path.append(pose.copy())
        if not c.active:
            break
    return states, np.array(path), closest


def test_drives_through_the_gap_without_touching_either_box():
    cfg = Settings(goal_distance=5., speed=.3, avoidance_speed=.2)
    world = np.vstack([box(1.5, 0.), box(1.5, .85)])
    states, path, closest = simulate(world, cfg)
    assert states[-1] == 'DONE'
    assert 'GAP' in states and 'GAP_THROUGH' in states and 'TURN_OUT' not in states
    assert closest > .05                  # at least 5 cm between the footprint and any box
    passing = path[np.argmin(abs(path[:, 0] - 1.5))]
    assert .15 + cfg.half_width < passing[1] < .70 - cfg.half_width
    assert abs(path[-1, 1]) < .1          # back on the route at the end


def test_tight_gap_with_2_cm_margin_and_noisy_lidar_does_not_get_stuck():
    # 22 cm rover, 2 cm gap margin, 32 cm opening, LiDAR noise of 1 cm.
    cfg = Settings(goal_distance=5., speed=.3, avoidance_speed=.2, half_width=.11, half_length=.125,
                   side_margin=.06, gap_margin=.02)
    world = np.vstack([box(1.8, 0.), box(1.8, .62)])
    rng = np.random.default_rng(1)
    c = started(cfg)
    pose, v, w, states = np.zeros(3), 0., 0., []
    for i in range(2400):
        noisy = world + rng.normal(0, .01, world.shape)
        v, w = c.step(pose, view(noisy, pose, cfg), i * .05, w, v)
        pose = pose + np.array([v * math.cos(pose[2]) * .05, v * math.sin(pose[2]) * .05, w * .05])
        local = (world - pose[:2]) @ rotation(pose[2])
        assert np.maximum(abs(local[:, 0]) - cfg.half_length, abs(local[:, 1]) - cfg.half_width).min() > 0
        if not states or states[-1] != c.state:
            states.append(c.state)
        if not c.active:
            break
    assert states[-1] == 'DONE' and 'GAP_THROUGH' in states and 'STOP' not in states


def test_something_staying_in_the_gap_falls_back_to_stop():
    cfg = Settings(goal_distance=5.)
    c = started(cfg)
    c.state, c.leg_heading, c.leg_start, c.gap_through, c.gap_blocked_since = 'GAP_THROUGH', 0., np.zeros(2), 2., None
    blocker = box(.6, 0.)                                 # appears in the gap, in the rover's path
    for t in np.arange(0., 2.9, .1):
        c.step(np.zeros(3), view(blocker), t)
        assert c.state == 'GAP_THROUGH' and c.reason.startswith('Something in the gap')
    c.step(np.zeros(3), view(blocker), 3.1)
    assert c.state == 'STOP'


ROVER = dict(goal_distance=5., speed=.3, avoidance_speed=.2, half_width=.11, half_length=.125,
             side_margin=.06, gap_margin=.02)


def test_gap_side_box_nearer_than_route_box_is_driven_through():
    # The 10 m run: the box beside the gap is 0.45 m nearer than the box on the route.
    # The entry point must be in front of the nearer box, not only the route box.
    cfg = Settings(**ROVER)
    world = np.vstack([box(2.2, 0.), box(1.75, -.62)])     # opening y -0.47..-0.15 = 32 cm
    states, path, closest = simulate(world, cfg)
    assert states[-1] == 'DONE'
    assert 'GAP_THROUGH' in states and 'TURN_OUT' not in states
    assert closest > 0


def test_gap_approach_never_waits_forever_just_short_of_the_entry():
    # Rover 6 cm from its entry point with an obstacle 0.26 m ahead along its heading.
    cfg = Settings(**ROVER)
    c = started(cfg)
    c.state, c.leg_heading, c.gap_heading = 'GAP', 0., -.32
    c.gap_entry, c.gap_through, c.gap_blocked_since = np.array([.06 * math.cos(-.32), .06 * math.sin(-.32)]), 1.5, None
    pose = np.array([0., 0., -.32])
    obstacle = box(.125 + .26 + .15, -.17)                   # its face 0.26 m from the bumper
    c.step(pose, view(obstacle, pose, cfg), 0.1)
    assert c.state == 'GAP_ALIGN'
    np.testing.assert_allclose(c.leg_start, c.gap_entry)     # centre line kept


def test_blocked_sidestep_turns_back_to_replan_instead_of_waiting_for_the_timeout():
    # Run 10b: sidestepping right, something 0.29 m in front, waited 68 s until the timeout.
    cfg = Settings(**ROVER)
    c = started(cfg)
    c.state, c.leg_heading, c.turn_heading = 'SIDESTEP', 0., -math.pi / 2
    c.leg_start, c.side_length = np.zeros(2), .5
    pose = np.array([0., 0., -math.pi / 2])
    blocker = box(0., -(cfg.half_length + .29 + .15))      # face 0.29 m in front of the bumper
    for t in np.arange(0., 4.9, .1):
        c.step(pose, view(blocker, pose, cfg), t)
        assert c.state == 'SIDESTEP'
    c.step(pose, view(blocker, pose, cfg), 5.1)
    assert c.state == 'TURN_FORWARD' and c.turn_heading == 0.


def test_run_10d_box_corner_at_the_front_corner_backs_up_then_centres():
    # Run 10d: in the gap 10 cm right of its centre; the right box's corner at x 0.12,
    # y -0.10 (inside the 0.11 half-width) triggered the emergency stop for good.
    cfg = Settings(**ROVER)
    c = started(cfg)
    c.state, c.leg_heading, c.leg_start, c.gap_through, c.gap_blocked_since = 'GAP_THROUGH', 0., np.zeros(2), 1.5, None
    world = np.vstack([box(.12 + .15, -.10 - .15), box(.17 + .15, .29 + .15)])
    pose, t, backed = np.zeros(3), 0., 0.
    assert c.step(pose, view(world, pose, cfg), t) == (0., 0.)          # emergency: stop first
    while t < 8.:
        t += .05
        v, w = c.step(pose, view(world, pose, cfg), t)
        if v < 0:
            backed += -v * .05
        pose = pose + np.array([v * math.cos(pose[2]) * .05, v * math.sin(pose[2]) * .05, w * .05])
        if v > 0:
            break
    assert .02 < backed <= .31                                  # backed up a little, not more
    assert v > 0 and w > 0                                              # then drives on, steering left


def test_gap_centring_uses_the_lidar_not_the_drifted_odometry():
    # Odometry says the rover is on the planned centre line, the LiDAR says it is 8 cm right.
    cfg = Settings(**ROVER)
    c = started(cfg)
    c.state, c.leg_heading, c.leg_start, c.gap_through, c.gap_blocked_since = 'GAP_THROUGH', 0., np.zeros(2), 1.5, None
    world = np.vstack([box(.5, -.13 - .15 + .08 - .10), box(.5, .26 + .15 + .08)])   # sides at -0.15 and +0.34
    v, w = c.step(np.zeros(3), view(world, cfg=cfg), 0.)
    assert v > 0 and w > 0                                              # steers left, towards the measured middle


def test_turn_finishes_despite_gyro_drift_at_the_tolerance_edge():
    cfg = Settings(**ROVER)
    c = started(cfg)
    c.state, c.turn_heading, c.leg_heading = 'TURN_FORWARD', 0., 0.
    heading, t = -.045, 0.                                              # just outside the 0.04 rad tolerance
    while c.state == 'TURN_FORWARD' and t < 10:
        v, w = c.step(np.array([0., 0., heading]), view(np.empty((0, 2)), cfg=cfg), t, 0.)
        heading += (w * .25 - math.radians(.5)) * .05                   # slow turf turn, drifting gyro
        t += .05
    assert c.state == 'PASS'


def test_no_gap_falls_back_to_the_sidestep_and_still_finishes():
    cfg = Settings(goal_distance=5., speed=.3, avoidance_speed=.2)
    states, _, closest = simulate(box(1.5, 0.), cfg)
    assert states[-1] == 'DONE'
    assert 'GAP' not in states and 'TURN_OUT' in states
    assert closest > .05


def test_far_obstacles_are_not_planned_as_a_gap():
    # Run 10e: from STOP it planned a "gap" in front of obstacles 8 m away.
    world = np.vstack([box(8., 0.), box(8., .85)])
    assert view(world).gap_plan(0.) is None


def test_box_beside_the_body_is_not_in_front():
    # Run 10e: a box 5 cm beside the rover's front half gave "Emergency front clearance".
    cfg = Settings(**ROVER)
    beside = box(-.05, cfg.half_width + .05 + .15)         # beside the body, 5 cm from its side, not past the bumper
    assert view(beside, cfg=cfg).clearance() == math.inf
    ahead = box(cfg.half_length + .05 + .15, 0.)            # face 5 cm ahead of the bumper
    assert abs(view(ahead, cfg=cfg).clearance() - .05) < .011


def wall(x, y0, y1):
    """A thin wall across the route at x, from y0 to y1, one point per centimetre."""
    ys = np.linspace(y0, y1, int((y1 - y0) * 100) + 1)
    return np.column_stack((np.full_like(ys, x), ys))


def test_both_sides_blocked_looks_around_and_still_reaches_the_goal():
    # Neither sidestep fits (a 60 cm box, sidesteps limited to 0.5 m): instead of waiting
    # until the timeout, it looks around, moves 0.5 m towards a clear direction, and goes on.
    cfg = Settings(**dict(ROVER, sidestep_max=.5))
    states, path, closest = simulate(box(1.8, 0., size=.60), cfg, seconds=170.)
    assert 'LOOK' in states and 'ESCAPE' in states
    assert states[-1] == 'DONE'
    assert closest > 0


def test_look_scan_turns_to_the_clear_direction_nearest_the_route():
    # 60 cm box ahead (no sidestep fits) and a box to the front right: from the scan,
    # standing still, the first clear direction is a small angle to the left.
    cfg = Settings(**dict(ROVER, sidestep_max=.5))
    c = started(cfg)
    c.state, c.leg_heading, c.settled_since = 'STOP', 0., -1.
    pose = np.zeros(3)
    world = np.vstack([box(cfg.half_length + .30 + .30, 0., size=.60), box(.55, -.75)])
    for t in np.arange(0., 5.2, .1):
        c.step(pose, view(world, pose, cfg), t)
    assert c.state == 'LOOK' and c.look_target is not None                # turning to a chosen direction
    angle = math.degrees(c.turn_heading)
    assert 0 < angle < 90 and angle % 5 == pytest.approx(0, abs=1e-6)     # left, a 5 degree step
    # every smaller angle on either side was blocked
    v = view(world, pose, cfg)
    for deg in range(0, int(round(angle)), 5):
        for sign in (1, -1):
            r = math.radians(sign * deg)
            assert not (v.clearance(r) > cfg.stop_clearance + .5 and v.segment_clear(r, .5))



def test_rejoin_turn_blocked_by_something_beside_turns_while_moving():
    # Run 10f: after the gap, REJOIN needed a turn towards the goal with something inside
    # the turning circle, and waited silently for good.
    cfg = Settings(**ROVER)
    c = started(cfg)
    c.state, c.goal = 'REJOIN', np.array([5., 2.])          # goal about 24 degrees to the left
    pose = np.array([1., 0., 0.])
    beside = box(1. - .10, -(cfg.half_width + .04 + .15))     # box 4 cm beside the right rear
    v, w = c.step(pose, view(beside, pose, cfg), 0.)
    assert v > 0 and w > 0 and 'turning while moving' in c.reason


def test_turn_blocked_by_a_box_corner_ahead_backs_up_until_it_can_turn():
    # The corner of a box just right of ahead is inside the turning circle and the way
    # ahead is too short for an arc: back up a few cm, then turn towards the goal.
    cfg = Settings(**ROVER)
    c = started(cfg)
    c.state, c.goal = 'REJOIN', np.array([5., 2.])
    world = box(1. + .20 + .15, -.12 - .15 + .10)               # corner at about (0.20, -0.02) from the centre
    pose, t, backed, turned = np.array([1., 0., 0.]), 0., 0., 0.
    while t < 8.:
        v, w = c.step(pose, view(world, pose, cfg), t)
        backed += max(0., -v) * .05; turned += w * .05
        pose = pose + np.array([v * math.cos(pose[2]) * .05, v * math.sin(pose[2]) * .05, w * .05])
        t += .05
    assert 0. < backed < .10 and turned > .1 and c.state != 'FAULT'


def test_boxed_in_keeps_looking_instead_of_stopping():
    # Boxes ahead, beside and behind: no turn, no arc, no room to back up. It must keep
    # trying (looking around) rather than FAULT.
    cfg = Settings(**ROVER)
    c = started(cfg)
    c.state, c.goal = 'REJOIN', np.array([5., 2.])
    world = np.vstack([box(.125 + .05 + .15, 0.), box(0., -.11 - .04 - .15), box(-.125 - .05 - .15, 0.)])
    pose, t, states = np.zeros(3), 0., set()
    while t < 60.:
        c.step(pose, view(world, pose, cfg), t); states.add(c.state); t += .05
    assert c.active and 'LOOK' in states



def test_emergency_outside_a_gap_backs_up_then_replans():
    # Run 10g: REJOIN with something 5 cm ahead of the bumper waited for good.
    cfg = Settings(**ROVER)
    c = started(cfg)
    c.state = 'REJOIN'
    world = box(cfg.half_length + .05 + .15, 0.)              # face 5 cm ahead of the bumper
    pose, t, backed, states = np.zeros(3), 0., 0., []
    while t < 12.:
        v, w = c.step(pose, view(world, pose, cfg), t)
        backed += max(0., -v) * .05
        pose = pose + np.array([v * math.cos(pose[2]) * .05, v * math.sin(pose[2]) * .05, w * .05])
        if not states or states[-1] != c.state:
            states.append(c.state)
        t += .05
    assert .05 < backed <= .31
    assert 'STOP' in states and c.state != 'FAULT'            # re-planned instead of waiting


def test_something_moving_with_the_rover_faults_with_a_reason():
    cfg = Settings(**ROVER)
    c = started(cfg)
    c.state = 'REJOIN'
    cable = np.array([[cfg.half_length + .03, 0.]])         # always 3 cm ahead, whatever the rover does
    pose, t = np.zeros(3), 0.
    while c.active and t < 15.:
        v, w = c.step(pose, view(np.repeat(cable, 5, axis=0), cfg=cfg), t)    # the cable stays put relative to the rover
        pose = pose + np.array([v * .05, 0., 0.])
        t += .05
    assert c.state == 'FAULT' and 'cable' in c.reason


def test_look_around_cycles_without_stopping_until_something_clears():
    # Nothing clear in any of the three directions for a while, then the way behind opens.
    cfg = Settings(**dict(ROVER, sidestep_max=.5, mission_timeout=600.))
    c = started(cfg)
    c.state, c.leg_heading, c.settled_since = 'STOP', 0., -1.
    pose, v, w, t = np.zeros(3), 0., 0., 0.
    ring = np.vstack([wall(.45, -.6, .6), wall(-.45, -.6, .6),
                      np.column_stack((np.linspace(-.45, .45, 91), np.full(91, .45))),
                      np.column_stack((np.linspace(-.45, .45, 91), np.full(91, -.45)))])
    looks = 0
    while t < 120.:
        world = ring if t < 60. else ring[ring[:, 0] > -.40]   # after 60 s the wall behind is gone
        v, w = c.step(pose, view(world, pose, cfg), t, w, v)
        looks += c.reason.startswith('No clear direction in the scan')
        pose = pose + np.array([v * math.cos(pose[2]) * .05, v * math.sin(pose[2]) * .05, w * .05])
        t += .05
        if c.state == 'ESCAPE':
            break
    assert c.active and looks > 3 and c.state == 'ESCAPE' and t > 60.
