"""Native AprilTag pose, calibration and robust range regression tests."""

from pathlib import Path
import math

import cv2
import numpy as np
import pytest

from l2s1_marker_detection.camera_model import CameraModel
from l2s1_marker_detection.native_apriltag import _load_extension, estimate_pose
from l2s2_localisation.geometry import MountModel, square_object_points
from l2s2_localisation.ranging import PoseHistory, SpatialFilter, base_to_map, observe_tag
from l2s2_localisation.range_node import load_config

CONFIG = Path(__file__).resolve().parents[1] / 'config/marker_ranges.yaml'
CALIBRATION = Path(__file__).resolve().parents[2] / 'ugv_main/ugv_vision/config/camera_info.yaml'


@pytest.fixture
def native():
    module = _load_extension()
    assert hasattr(module, 'estimate_pose'), 'Build apriltag with the new pose binding first'
    return module


@pytest.mark.parametrize('rectified,position', [(False, (0, 0, 2)), (False, (.3, -.1, 3)),
                                               (True, (0, 0, 3.2)), (True, (-.5, .2, 2.5))])
def test_native_pose_recovers_metric_translation(native, rectified, position):
    camera = CameraModel.from_yaml(CALIBRATION, rectified=rectified)
    corners, _ = cv2.projectPoints(square_object_points(.14), np.array([math.pi-.2, .1, .05]),
                                   np.array(position, float), camera.K, camera.D)
    pose = estimate_pose(corners, .14, camera.K, camera.D)
    assert pose['translation'] == pytest.approx(position, abs=2e-4)
    assert pose['reprojection_error_px'] < .001
    wrong_size = estimate_pose(corners, .30, camera.K, camera.D)
    assert wrong_size['translation'][2]/pose['translation'][2] == pytest.approx(.30/.14, rel=1e-5)


def test_rectified_projection_uses_p_and_zero_distortion():
    raw = CameraModel.from_yaml(CALIBRATION)
    rect = CameraModel.from_yaml(CALIBRATION, rectified=True)
    assert raw.K[0, 0] == pytest.approx(289.11451)
    assert rect.K[0, 0] == pytest.approx(196.89772)
    assert np.count_nonzero(rect.D) == 0


def test_pose_translation_becomes_robot_centre_horizontal_range(native):
    camera = CameraModel.from_yaml(CALIBRATION, rectified=True)
    t = np.array([-.3, .05, 3.0])
    corners, _ = cv2.projectPoints(square_object_points(.14), np.array([math.pi-.1, 0, 0]),
                                   t, camera.K, camera.D)
    item = {'corners': corners.reshape(4, 2), 'hamming': 0, 'decision_margin': 80}
    observation = observe_tag(item, .14, camera, (640, 480), MountModel(), 0, 0, load_config(CONFIG))
    assert observation['position'] == pytest.approx([3.04, .3, .161], abs=2e-4)
    assert observation['camera_distance_m'] == pytest.approx(np.linalg.norm(t), rel=1e-4)


def test_corner_and_decoder_quality_checks(native):
    camera = CameraModel.from_yaml(CALIBRATION)
    config = load_config(CONFIG)
    for item in ({'hamming': 2, 'corners': []}, {'decision_margin': 1, 'corners': []}):
        with pytest.raises(ValueError):
            observe_tag(item, .14, camera, (640, 480), MountModel(), 0, 0, config)
    with pytest.raises(ValueError):
        estimate_pose(np.zeros((4, 2)), .14, camera.K)
    with pytest.raises(ValueError):
        native.estimate_pose(np.zeros((4, 2)), np.zeros((3, 3)), .14, 200, 200, 320, 240)


def test_time_interpolation_wraps_heading_and_rejects_old_pose():
    history = PoseHistory(.25)
    history.add(10, (0, 0, math.radians(179)))
    history.add(10.2, (1, 2, math.radians(-179)))
    pose = history.at(10.1)
    assert pose[:2] == pytest.approx([.5, 1])
    assert abs(pose[2]) == pytest.approx(math.pi)
    assert history.at(11) is None


def test_spatial_filter_rejects_outlier_and_releases_after_gap():
    filtering = SpatialFilter(9, 1)
    for i, x in enumerate([3.0, 3.1, 2.9]):
        median, _ = filtering.add(10+i*.1, [x, .75, .2], 'map:1')
    with pytest.raises(ValueError):
        filtering.add(10.3, [8, .75, .2], 'map:1')
    median, _ = filtering.add(12, [8, .75, .2], 'map:1')
    assert median == pytest.approx([8, .75, .2])
    median, points = filtering.add(12.1, [2, 0, .2], 'map:2')
    assert len(points) == 1


def test_world_filter_does_not_lag_robot_motion():
    filtering = SpatialFilter()
    for i in range(6):
        robot = (-3.2+i*.1, 0, 0)
        relative = [0-robot[0], .75, .2]
        median, _ = filtering.add(10+i*.1, base_to_map(relative, robot), 'map:1')
    assert median == pytest.approx([0, .75, .2])
    assert math.hypot(median[0]-robot[0], median[1]-robot[1]) == pytest.approx(math.hypot(2.7, .75))
