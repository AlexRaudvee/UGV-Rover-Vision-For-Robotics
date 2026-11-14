"""Regression for the actual lab markers missed by the OpenCV-only pipeline."""
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np
import pytest

from l2s1_marker_detection.aruco_compat import normalize_family
from l2s1_marker_detection.detection import MarkerDetector, PRESETS
from l2s1_marker_detection.native_apriltag import _load_extension


def test_standard_family_aliases():
    assert normalize_family('tagStandard41h12') == 'APRILTAG_standard41h12'
    assert normalize_family('APRILTAG_standard41h12') == 'APRILTAG_standard41h12'


@pytest.mark.parametrize('preset', ['baseline', 'improved', 'upscale'])
def test_real_lab_markers(preset):
    try:
        _load_extension()
    except ImportError as error:
        pytest.skip(str(error))
    image = cv2.imread(str(Path(__file__).parent / 'data' / 'lab_standard41h12.png'))
    detector = MarkerDetector(replace(PRESETS[preset], families=('tagStandard41h12',)))
    detections = detector.detect(image).detections
    assert {(d.family, d.marker_id) for d in detections} == {
        ('APRILTAG_standard41h12', 1), ('APRILTAG_standard41h12', 2)}
    # The family measures the inner border, not the entire printed outer pattern.
    for detection in detections:
        assert 15 < detection.side_px < 45
        assert np.all(np.cross(detection.corners[1] - detection.corners[0],
                               detection.corners[2] - detection.corners[1]) > 0)
