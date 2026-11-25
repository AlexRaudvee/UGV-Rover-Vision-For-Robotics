"""Navigation evidence must distinguish measured floor, obstacles and unknown."""

from types import SimpleNamespace

import numpy as np
import pytest

from detection_improved.floor import FloorPlane
from detection_improved.navigation import evidence, timing


def scene():
    # Every pixel at y=1 is on a level floor one metre below this optical origin.
    depth = np.ones((4, 4), dtype=float)
    matrix = np.array([[2., 0, 1.5], [0, 2., -.5], [0, 0, 1.]])
    plane = FloorPlane(np.array([0., -1., 0.]), .25, 'fit')
    return depth, matrix, plane


def test_empty_or_missing_depth_does_not_create_free_space():
    depth, matrix, plane = scene()
    out = evidence(np.full_like(depth, np.nan), matrix, plane, np.eye(3), step=1)
    assert out['free_points'] == out['occupied_points'] == []
    assert out['unknown_pixels'] == depth.size
    out = evidence(depth, matrix, None, np.eye(3), step=1)
    assert not out['usable'] and not out['free_points']


def test_only_measured_floor_patches_are_free_and_holes_stay_unknown():
    depth, matrix, plane = scene()
    support = np.ones_like(depth, dtype=np.uint8)
    out = evidence(depth, matrix, plane, np.eye(3), support, step=1)
    assert len(out['free_points']) == 4  # row zero is at floor height
    support[0, :] = 0  # WLS-filled pixels, even though metric depth is finite.
    blocked = evidence(depth, matrix, plane, np.eye(3), support, step=1)
    assert blocked['free_points'] == []
    assert blocked['unknown_pixels'] == 4


def test_unmeasured_silhouette_does_not_clear_its_angular_sector():
    depth, matrix, plane = scene()
    out = evidence(depth, matrix, plane, np.eye(3), step=1,
                   unknown=[dict(angle_min=-1.5, angle_max=1.5)])
    assert out['free_points'] == []


def test_out_of_target_width_and_clipped_objects_still_produce_obstacle_faces():
    depth, matrix, plane = scene()
    blocker = SimpleNamespace(distance_m=1.5, bearing=0., angle_min=-.2, angle_max=.2,
                              range_std_m=.03, width_std_m=.02, accepted=False, clipped=True)
    out = evidence(depth, matrix, plane, np.eye(3), blockers=[blocker], step=1)
    assert len(out['blocker_faces']) == 1
    assert out['blocker_faces'][0]['target_size'] is False
    assert out['blocker_faces'][0]['clipped'] is True
    assert len(out['blocker_faces'][0]['points']) == 2


def test_pitch_compensation_is_applied_once_before_tf():
    depth, matrix, plane = scene()
    rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    baseline = evidence(depth, matrix, plane, np.eye(3), step=1)
    corrected = evidence(depth, matrix, plane, rotation, step=1)
    assert np.array(corrected['free_points']) == pytest.approx(np.array(baseline['free_points'])@rotation)


@pytest.mark.parametrize('rgb,depth,now,valid', [
    (10., 10., 10.2, True), (10., 10.1, 10.3, True),
    (10., 10., 12., False), (10., 9.5, 10.1, False),
    (0., 10., 10., False), (11., 11., 10., False),
])
def test_navigation_checks_acquisition_times_even_when_display_is_receipt_based(rgb, depth, now, valid):
    assert timing(rgb, depth, now)['usable'] is valid
