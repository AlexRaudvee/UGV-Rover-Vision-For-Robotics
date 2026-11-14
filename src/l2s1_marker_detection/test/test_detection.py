"""Detection tests on synthetic marker images (pure Python, no ROS)."""

from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np
import pytest

from l2s1_marker_detection import aruco_compat as compat
from l2s1_marker_detection.camera_model import CameraModel
from l2s1_marker_detection.detection import (DEFAULT_FAMILIES, PRESETS, Detection, MarkerDetector,
                                           draw, merge_duplicates)
from l2s1_marker_detection.evaluate import main as evaluate_main

CAMERA_INFO = (Path(__file__).resolve().parents[2]
               / 'ugv_main' / 'ugv_vision' / 'config' / 'camera_info.yaml')


def scene(family='APRILTAG_36h11', marker_id=5, side=120, center=(320, 240), blur=0.0,
          low=0, high=255, canvas=None):
    """Paste a marker with a white quiet zone; optional blur and reduced contrast."""
    image = np.full((480, 640), high, np.uint8) if canvas is None else canvas
    tag = compat.marker_image(family, marker_id, side).astype(float)
    tag = (low + tag / 255.0 * (high - low)).astype(np.uint8)
    x0, y0 = int(center[0] - side / 2), int(center[1] - side / 2)
    image[y0:y0 + side, x0:x0 + side] = tag
    if blur:
        image = cv2.GaussianBlur(image, (0, 0), blur)
    return image


def found(detector, image):
    return [(d.family, d.marker_id) for d in detector.detect(image).detections]


def test_detects_marker_category_and_id():
    detector = MarkerDetector(PRESETS['baseline'])
    result = detector.detect(scene(side=120))
    assert [(d.family, d.marker_id) for d in result.detections] == [('APRILTAG_36h11', 5)]
    assert np.allclose(result.detections[0].center, (320, 240), atol=1.5)
    assert result.detections[0].side_px == pytest.approx(120, abs=3)


@pytest.mark.parametrize('family', DEFAULT_FAMILIES)
def test_each_family_reports_its_own_category(family):
    assert found(MarkerDetector(PRESETS['baseline']), scene(family, 3)) == [(family, 3)]


def test_several_families_in_one_image():
    image = scene('APRILTAG_36h11', 1, side=100, center=(160, 240))
    image = scene('6X6_250', 17, side=100, center=(480, 240), canvas=image)
    assert sorted(found(MarkerDetector(PRESETS['improved']), image)) == [
        ('6X6_250', 17), ('APRILTAG_36h11', 1)]


def test_family_aliases():
    assert compat.normalize_family('tag36h11') == 'APRILTAG_36h11'
    assert compat.normalize_family('apriltag_25H9') == 'APRILTAG_25h9'
    assert compat.normalize_family('6x6_250') == '6X6_250'
    with pytest.raises(ValueError):
        compat.normalize_family('not_a_family')


def test_upscaled_corners_are_in_original_pixels():
    image = scene(side=90, center=(300, 220))
    base = MarkerDetector(PRESETS['baseline']).detect(image).detections[0]
    upscaled = MarkerDetector(PRESETS['upscale']).detect(image).detections[0]
    assert np.abs(base.corners - upscaled.corners).max() < 1.5


def test_improved_never_loses_and_sometimes_gains():
    """Over small, blurred and low-contrast cases, improved >= baseline everywhere."""
    cases = ([dict(side=s) for s in (12, 14, 16, 20)]
             + [dict(side=60, blur=b) for b in (2, 3, 4)]
             + [dict(side=80, low=lo, high=hi) for lo, hi in ((110, 130), (115, 125))])
    baseline = MarkerDetector(replace(PRESETS['baseline'], families=('APRILTAG_36h11',)))
    improved = MarkerDetector(replace(PRESETS['improved'], families=('APRILTAG_36h11',)))
    base_hits = [len(found(baseline, scene(**case))) for case in cases]
    improved_hits = [len(found(improved, scene(**case))) for case in cases]
    assert all(i >= b for i, b in zip(improved_hits, base_hits))
    assert sum(improved_hits) > sum(base_hits)


def test_merge_keeps_one_detection_per_physical_marker():
    square = np.float32([[0, 0], [40, 0], [40, 40], [0, 40]])
    found_items = [((1, 0), Detection('B', 2, square + 1)),
                   ((0, 1), Detection('A', 1, square)),
                   ((0, 0), Detection('A', 1, square + 100))]
    kept = merge_duplicates(found_items)
    assert [(d.family, d.marker_id) for d in kept] == [('A', 1), ('A', 1)]


def test_draw_returns_colour_image_of_same_size():
    image = scene()
    detections = MarkerDetector(PRESETS['baseline']).detect(image).detections
    canvas = draw(image, detections, 'status')
    assert canvas.shape == (480, 640, 3)


@pytest.mark.skipif(not CAMERA_INFO.exists(), reason='ugv_vision camera_info.yaml not found')
def test_distort_points_matches_undistort_remap():
    """undistort() samples raw pixel map(u, v) for output pixel (u, v); invert exactly that."""
    camera = CameraModel.from_yaml(CAMERA_INFO)
    map_x, map_y = cv2.initUndistortRectifyMap(camera.K, camera.D, None, camera.K,
                                               (640, 480), cv2.CV_32FC1)
    pixels = np.float32([[60, 50], [320, 240], [590, 420], [120, 400], [600, 60]])
    expected = np.float32([[map_x[int(v), int(u)], map_y[int(v), int(u)]] for u, v in pixels])
    assert np.abs(camera.distort_points(pixels, (640, 480)) - expected).max() < 0.05


@pytest.mark.skipif(not CAMERA_INFO.exists(), reason='ugv_vision camera_info.yaml not found')
def test_undistort_preset_reports_raw_coordinates():
    camera = CameraModel.from_yaml(CAMERA_INFO)
    image = scene(side=100, center=(320, 240))
    base = MarkerDetector(PRESETS['baseline']).detect(image).detections[0]
    undistorted = MarkerDetector(PRESETS['undistort'], camera).detect(image).detections[0]
    assert np.abs(base.center - undistorted.center).max() < 2.0


def test_evaluate_cli_writes_csv(tmp_path):
    cv2.imwrite(str(tmp_path / 'a.png'), scene(marker_id=4))
    out = tmp_path / 'eval'
    assert evaluate_main([str(tmp_path), '--presets', 'baseline', 'improved',
                          '--expect', 'tag36h11:4', '--repeat', '1', '--out', str(out)]) == 0
    text = (out / 'detections.csv').read_text()
    assert 'baseline' in text and '1/1' in text
    assert (out / 'annotated' / 'improved' / 'a.jpg').exists()
