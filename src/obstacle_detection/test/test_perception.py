"""Detection regressions with independently constructed camera scenes."""

import math
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from obstacle_detection.perception import CameraModel, DetectorSettings, detect_black_objects


def camera_info():
    return SimpleNamespace(
        width=640, height=480, distortion_model='plumb_bob', d=[0.0] * 5,
        k=[500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0],
        p=[400.0, 0.0, 320.0, 0.0, 0.0, 400.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0])


def scene(center=90, color=(20, 20, 20)):
    image = np.full((240, 320, 3), (45, 150, 55), dtype=np.uint8)
    cv2.rectangle(image, (center - 25, 105), (center + 25, 200), color, -1)
    return image


def model():
    return CameraModel.from_info(camera_info(), 320, 240, rectified=False)


def test_calibration_uses_raw_k_or_rectified_p_and_scales_resolution():
    raw = model()
    rectified = CameraModel.from_info(camera_info(), 320, 240, rectified=True)
    assert raw.matrix[0, 0] == 250
    assert rectified.matrix[0, 0] == 200
    assert raw.matrix[0, 2] == rectified.matrix[0, 2] == 160


@pytest.mark.parametrize('center,sign', [(90, 1), (230, -1)])
def test_black_obstacle_bearing_and_extent(center, sign):
    detections, _, _, _ = detect_black_objects(scene(center), model(), DetectorSettings())
    assert len(detections) == 1
    obstacle = detections[0]
    assert sign * obstacle.bearing > 0
    assert obstacle.angle_min < obstacle.bearing < obstacle.angle_max
    assert obstacle.distance_source == 'size_estimate'
    assert 1.0 < obstacle.distance_m < 1.6


@pytest.mark.parametrize('color', [(0, 0, 220), (0, 0, 65), (210, 210, 210)])
def test_red_barriers_and_bright_objects_are_not_black_candidates(color):
    detections, _, _, _ = detect_black_objects(scene(color=color), model(), DetectorSettings())
    assert detections == []


def test_clear_field_and_small_speckle_are_ignored():
    image = np.full((240, 320, 3), (45, 150, 55), dtype=np.uint8)
    image[150:153, 140:143] = 0
    detections, _, _, _ = detect_black_objects(image, model(), DetectorSettings())
    assert detections == []


def test_depth_only_samples_the_obstacle_and_returns_radial_range():
    image = scene()
    depth = np.full(image.shape[:2], 4.0, dtype=np.float32)
    depth[105:201, 65:116] = 1.5
    depth[120:125, 80:85] = np.nan
    depth[130:135, 80:85] = 0
    detections, _, _, _ = detect_black_objects(image, model(), DetectorSettings(), depth)
    assert len(detections) == 1
    obstacle = detections[0]
    assert obstacle.distance_source == 'depth'
    assert obstacle.distance_m == pytest.approx(1.5 / math.cos(obstacle.bearing), rel=0.01)


def test_invalid_depth_is_explicitly_labeled_as_size_estimate():
    image = scene()
    depth = np.full(image.shape[:2], np.nan, dtype=np.float32)
    detections, _, _, _ = detect_black_objects(image, model(), DetectorSettings(), depth)
    assert detections[0].distance_source == 'size_estimate'
    assert math.isfinite(detections[0].distance_m)


def test_different_depth_geometry_is_rejected():
    with pytest.raises(ValueError, match='aligned'):
        detect_black_objects(scene(), model(), DetectorSettings(), np.ones((10, 10)))


def test_invalid_calibration_is_rejected():
    info = camera_info()
    info.k[0] = 0
    with pytest.raises(ValueError, match='focal'):
        CameraModel.from_info(info, 320, 240, rectified=False)
