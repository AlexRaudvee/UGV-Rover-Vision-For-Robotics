"""Check bin semantics and temporal behavior independently of ROS."""

import math

import numpy as np
import pytest

from obstacle_detection.histogram import PolarHistogram
from obstacle_detection.perception import Detection


def obstacle(low=10, high=25, distance=1.0):
    return Detection((0, 0, 10, 10), math.radians((low + high) / 2),
                     math.radians(low), math.radians(high), distance, 'depth', 1.0, 1000)


def test_unseen_is_distinct_from_searched_without_obstacles():
    histogram = PolarHistogram(smoothing_s=0)
    histogram.update([], (-math.pi / 4, math.pi / 4), now=1)
    assert np.isnan(histogram.values()[~histogram.observed]).all()
    assert (histogram.values()[histogram.observed] == 0).all()
    assert np.isinf(histogram.ranges[histogram.observed]).all()


def test_whole_obstacle_width_contributes_and_positive_bearing_is_left():
    histogram = PolarHistogram(smoothing_s=0)
    histogram.update([obstacle()], (-1, 1), now=1)
    hit = histogram.scores > 0
    assert hit.sum() >= 3
    assert (histogram.angles[hit] > 0).all()
    assert (histogram.ranges[hit] == 1).all()


def test_overlapping_obstacles_keep_the_nearest_range():
    histogram = PolarHistogram(smoothing_s=0)
    histogram.update([obstacle(distance=3), obstacle(distance=1)], (-1, 1), now=1)
    assert np.all(histogram.ranges[histogram.scores > 0] == 1)


def test_nearer_obstacles_have_higher_scores():
    near = PolarHistogram(smoothing_s=0)
    far = PolarHistogram(smoothing_s=0)
    near.update([obstacle(distance=0.5)], (-1, 1), now=1)
    far.update([obstacle(distance=4)], (-1, 1), now=1)
    assert near.scores.max() > far.scores.max() > 0


def test_left_and_right_obstacles_make_distinct_groups():
    histogram = PolarHistogram(smoothing_s=0)
    histogram.update([obstacle(15, 25), obstacle(-25, -15)], (-1, 1), now=1)
    assert np.any(histogram.scores[histogram.angles > 0] > 0)
    assert np.any(histogram.scores[histogram.angles < 0] > 0)
    assert np.all(histogram.scores[np.abs(histogram.angles) < math.radians(10)] == 0)


def test_removed_obstacle_releases_instead_of_becoming_a_permanent_ghost():
    histogram = PolarHistogram(smoothing_s=0.15, release_s=0.5)
    histogram.update([obstacle()], (-1, 1), now=1)
    histogram.update([], (-1, 1), now=1.1)
    assert histogram.scores.max() > 0
    histogram.update([], (-1, 1), now=1.6)
    assert histogram.scores.max() == 0
    assert np.isinf(histogram.ranges[histogram.observed]).all()


def test_stale_clear_invalidates_all_directions_immediately():
    histogram = PolarHistogram()
    histogram.update([obstacle()], (-1, 1), now=1)
    histogram.clear_unknown()
    assert np.isnan(histogram.values()).all()
    assert np.isnan(histogram.ranges).all()


@pytest.mark.parametrize('width', [0, -5, float('nan'), 7])
def test_invalid_bin_sizes_are_rejected(width):
    with pytest.raises(ValueError):
        PolarHistogram(bin_degrees=width)
