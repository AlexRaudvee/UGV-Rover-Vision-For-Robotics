"""Pieces of one object (e.g. a rover's deck and track) merge; separate objects do not."""

import math

from detection_improved.perception import Detection, DetectorSettings, merge_detections
import pytest

ON = DetectorSettings(merge_max_gap_px=15)


def piece(x, y, w, h, distance, left_y=-.15, right_y=.15, z=1.5):
    """Build a measured piece spanning lateral offsets left_y..right_y (m) at depth z."""
    low, high = math.atan(left_y / z), math.atan(right_y / z)
    return Detection((x, y, w, h), (low + high) / 2, low, high, distance, 'depth', 1.0,
                     float(w * h), right_y - left_y, z)


def test_vertically_stacked_pieces_at_same_distance_merge():
    deck = piece(300, 100, 60, 30, 1.50, -.10, .08)
    track = piece(290, 140, 90, 40, 1.55, -.14, .14)   # 10 px below, mostly overlapping
    merged = merge_detections([deck, track], ON)
    assert len(merged) == 1
    item = merged[0]
    assert item.bbox == (290, 100, 90, 80) and len(item.parts) == 2
    assert item.distance_m == pytest.approx(1.50)          # nearer piece
    assert item.width_m == pytest.approx(0.28, abs=1e-6)   # combined -0.14..0.14 m


def test_pieces_at_different_distances_or_side_by_side_stay_separate():
    near, far = piece(300, 100, 60, 30, 1.5), piece(300, 140, 60, 30, 2.5)
    assert len(merge_detections([near, far], ON)) == 2
    left, right = piece(100, 100, 60, 60, 1.5), piece(300, 100, 60, 60, 1.5)
    assert len(merge_detections([left, right], ON)) == 2
    high, low = piece(300, 0, 60, 30, 1.5), piece(300, 200, 60, 30, 1.5)   # far apart
    assert len(merge_detections([high, low], ON)) == 2


def test_three_pieces_merge_and_merging_can_be_disabled():
    pieces = [piece(300, 100, 60, 20, 1.5), piece(300, 125, 60, 20, 1.5),
              piece(300, 150, 60, 20, 1.6)]
    assert len(merge_detections(pieces, ON)) == 1
    assert len(merge_detections(pieces, DetectorSettings())) == 3      # default: off
