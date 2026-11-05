"""Metric plot axes and robot-relative cardinal directions."""

import math

import pytest

from polar_hist.histogram import PolarHistogram
from polar_hist.visualization import display_degrees, histogram_image
from object_detection.perception import Detection


@pytest.mark.parametrize('bearing,expected', [(-90, 0), (0, 90), (90, 180), (180, 270)])
def test_robot_angle_zero_is_right(bearing, expected):
    assert display_degrees(math.radians(bearing)) == pytest.approx(expected)


def test_camera_yaw_is_accounted_for():
    assert display_degrees(0, math.pi / 2) == pytest.approx(180)


def test_bar_height_is_metric_distance_not_proximity_score():
    histogram = PolarHistogram(bin_degrees=2, smoothing_s=0)
    item = Detection((0, 0, 1, 1), 0., -.06, .06, 1.5, 'depth', 1., 100)
    histogram.update([item], (-.5, .5), now=1)
    image = histogram_image(histogram, [], 'TEST', '1.5 meters', display_range_m=3.)
    # At 90 degrees (front), 1.5 m reaches half of the 0..3 m plot height.
    x = round(565 + 90 / 360 * 500)
    assert image[320, x, 2] > 200
    assert image[180, x, 2] < 100
    assert image.shape == (620, 1100, 3)
