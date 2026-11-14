"""Pan-tilt geometry: project known markers through the real calibration, recover them."""

import math
from pathlib import Path

import cv2
import numpy as np
import pytest

from l2s1_marker_detection.camera_model import CameraModel
from l2s2_localisation.geometry import (MountModel, observe, square_object_points,
                                          summarize)

CAMERA_INFO = (Path(__file__).resolve().parents[2]
               / 'ugv_main' / 'ugv_vision' / 'config' / 'camera_info.yaml')
pytestmark = pytest.mark.skipif(not CAMERA_INFO.exists(), reason='camera_info.yaml not found')
SIZE = 0.15
MOUNT = MountModel()


def project_marker(marker_xyz, pan, tilt, camera, noise_px=0.0, seed=0):
    """Pixel corners of an upright marker at marker_xyz (robot frame) facing the camera."""
    rotation = MOUNT.camera_rotation(pan, tilt)          # optical -> robot
    centre = rotation.T @ (np.asarray(marker_xyz) - MOUNT.camera_center(pan, tilt))
    # Marker axes in the optical frame: x right, y up (= -optical y), z towards camera.
    toward = -centre / np.linalg.norm(centre)
    up = np.array([0.0, -1.0, 0.0])
    x_axis = np.cross(up, toward)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(toward, x_axis)
    marker_rotation = np.column_stack([x_axis, y_axis, toward])
    rvec, _ = cv2.Rodrigues(marker_rotation)
    pixels, _ = cv2.projectPoints(square_object_points(SIZE), rvec, centre, camera.K, camera.D)
    pixels = pixels.reshape(4, 2)
    if noise_px:
        pixels += np.random.default_rng(seed).normal(0, noise_px, pixels.shape)
    return pixels


@pytest.fixture(scope='module')
def camera():
    return CameraModel.from_yaml(CAMERA_INFO)


@pytest.mark.parametrize('marker, pan_deg, tilt_deg', [
    ((2.0, 0.0, 0.25), 0, 0),        # straight ahead
    ((1.5, 1.0, 0.30), 30, 0),       # front left, camera panned left
    ((1.0, -2.0, 0.20), -60, 0),     # right, camera panned right
    ((-1.5, 1.5, 0.40), 135, 5),     # behind left, camera tilted up
    ((2.5, 0.5, 0.25), 0, 0),        # off-centre in the image
])
def test_recovers_marker_position(camera, marker, pan_deg, tilt_deg):
    pan, tilt = math.radians(pan_deg), math.radians(tilt_deg)
    corners = project_marker(marker, pan, tilt, camera)
    sighting = observe('APRILTAG_36h11', 1, corners, SIZE, camera, (640, 480), MOUNT, pan, tilt)
    assert np.allclose(sighting.position, marker, atol=0.01)
    assert sighting.range == pytest.approx(math.hypot(marker[0], marker[1]), abs=0.01)
    assert sighting.bearing == pytest.approx(math.atan2(marker[1], marker[0]), abs=math.radians(0.3))


def test_left_is_positive_bearing(camera):
    corners = project_marker((2.0, 0.6, 0.25), 0.0, 0.0, camera)
    sighting = observe('APRILTAG_36h11', 1, corners, SIZE, camera, (640, 480), MOUNT, 0.0, 0.0)
    assert sighting.bearing > 0
    assert corners[:, 0].mean() < 320          # appears in the left half of the image


def test_wrong_marker_size_scales_distance(camera):
    """The Session 2 hint: distance is proportional to the assumed marker size."""
    corners = project_marker((2.0, 0.0, 0.25), 0.0, 0.0, camera)
    true = observe('T', 1, corners, SIZE, camera, (640, 480), MOUNT, 0.0, 0.0)
    wrong = observe('T', 1, corners, SIZE * 1.2, camera, (640, 480), MOUNT, 0.0, 0.0)
    assert wrong.camera_distance == pytest.approx(true.camera_distance * 1.2, rel=0.01)


def test_summary_prefers_pan_stop_with_marker_near_image_centre(camera):
    marker = (1.5, 1.2, 0.25)
    sightings = []
    for pan_deg in (0, 60):            # marker at ~39 deg: visible at both stops
        pan = math.radians(pan_deg)
        for seed in range(5):
            corners = project_marker(marker, pan, 0.0, camera, noise_px=0.3, seed=seed)
            sightings.append(observe('APRILTAG_36h11', 7, corners, SIZE, camera, (640, 480),
                                     MOUNT, pan, 0.0))
    [entry] = summarize(sightings)
    expected_pan = min((0, 60), key=lambda p: abs(math.degrees(math.atan2(1.2, 1.5)) - p))
    assert entry['pan_deg'] == expected_pan
    assert entry['seen_at_pans_deg'] == [0.0, 60.0]
    # 0.3 px corner noise on a ~23 px tag already gives a few cm of range error.
    assert entry['range_m'] == pytest.approx(math.hypot(1.5, 1.2), abs=0.06)
    assert entry['frames'] == 5
