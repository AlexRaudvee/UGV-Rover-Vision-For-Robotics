"""Floor plane fit from synthetic depth images."""

import math

import numpy as np
from detection_improved.floor import (classify, depth_points, fit_floor, FloorTracker,
                                      prior_plane)

MATRIX = np.array([[300.0, 0, 160], [0, 300.0, 120], [0, 0, 1]])
WIDTH, HEIGHT = 320, 240


def camera_pose(height_m, pitch_down_deg):
    """Return (rotation, translation): p_camera = R p_floor + t, floor frame z up."""
    phi = math.radians(pitch_down_deg)
    # Level camera: optical x = -floor y, optical y = -floor z, optical z = floor x.
    level = np.array([[0.0, -1, 0], [0, 0, -1], [1, 0, 0]])
    tilt = np.array([[1.0, 0, 0], [0, math.cos(phi), -math.sin(phi)],
                     [0, math.sin(phi), math.cos(phi)]])
    rotation = tilt @ level
    return rotation, -rotation @ np.array([0.0, 0, height_m])


def floor_depth(height_m, pitch_down_deg, box=None):
    """Render optical-Z depth of a flat floor, optionally with a box face standing on it."""
    rotation, translation = camera_pose(height_m, pitch_down_deg)
    plane = prior_plane(rotation, translation)
    u, v = np.meshgrid(np.arange(WIDTH), np.arange(HEIGHT))
    rays = np.dstack(((u - MATRIX[0, 2]) / MATRIX[0, 0], (v - MATRIX[1, 2]) / MATRIX[1, 1],
                      np.ones(u.shape)))
    along = rays @ plane.normal
    with np.errstate(divide='ignore', invalid='ignore'):
        depth = np.where(along < -1e-6, -plane.offset / along, np.nan)
    if box is not None:
        x0, x1, y0, y1, z = box
        depth[y0:y1, x0:x1] = z
    return depth


def test_fit_recovers_true_floor_from_wrong_prior():
    depth = floor_depth(0.20, 8.0)
    prior = prior_plane(*camera_pose(0.25, 3.0))   # 5 cm and 5 degrees off
    plane = fit_floor(depth_points(depth, MATRIX), prior)
    assert plane is not None and plane.source == 'fit'
    assert abs(plane.camera_height_m - 0.20) < 0.005
    assert abs(plane.pitch_down_deg - 8.0) < 0.3


def test_box_is_above_floor_and_floor_is_floor():
    depth = floor_depth(0.20, 5.0, box=(140, 180, 60, 110, 1.5))
    points = depth_points(depth, MATRIX)
    plane = fit_floor(points, prior_plane(*camera_pose(0.20, 5.0)))
    floor, above = classify(plane.height(points))
    assert above[65:105, 145:175].all()
    assert floor[200:, :].all()


def test_below_floor_is_floor_unless_far_below():
    floor, above = classify(np.array([-0.30, -0.10, 0.0, 0.03, 0.05, np.nan]))
    assert floor.tolist() == [False, True, True, True, False, False]
    assert above.tolist() == [False, False, False, False, True, False]


def test_wall_only_is_rejected_and_tracker_falls_back():
    depth = np.full((HEIGHT, WIDTH), 1.0)   # camera facing a wall: no floor
    prior = prior_plane(*camera_pose(0.20, 5.0))
    assert fit_floor(depth_points(depth, MATRIX), prior) is None
    tracker = FloorTracker()
    assert tracker.update(depth_points(depth, MATRIX), prior) is None
    good = depth_points(floor_depth(0.20, 5.0), MATRIX)
    assert tracker.update(good, prior).source == 'fit'
    for _ in range(tracker.settings.hold_frames):
        assert tracker.update(depth_points(depth, MATRIX), prior).source == 'held'
    assert tracker.update(depth_points(depth, MATRIX), prior) is None
