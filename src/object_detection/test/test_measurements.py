"""Independent physical scenes verify width, color, and depth provenance."""

import math
from types import SimpleNamespace

import numpy as np
from object_detection.perception import (
    CameraModel, depth_for_blob, DetectorSettings, find_black_blobs,
    lidar_for_blobs, measure_blob, scan_points_in_camera,
)
import pytest


def camera():
    return CameraModel(640, 480, np.array([[500., 0, 320], [0, 500, 240], [0, 0, 1]]),
                       np.zeros(5))


def scene(width=90, center=320, color=20):
    image = np.full((480, 640, 3), 200, np.uint8)
    x = center - width // 2
    image[180:340, max(0, x):min(640, x + width)] = color
    return image


@pytest.mark.parametrize('pixels,expected,accepted', [
    (60, 0.18, False), (77, 0.231, True), (90, 0.27, True),
    (100, 0.30, True), (110, 0.33, False),
])
def test_measured_width_filter(pixels, expected, accepted):
    settings = DetectorSettings()
    blobs, _ = find_black_blobs(scene(pixels), camera(), settings)
    assert len(blobs) == 1
    item = measure_blob(blobs[0], 1.5, 'depth', settings)
    assert item.width_m == pytest.approx(expected, abs=1e-6)
    assert item.accepted is accepted


@pytest.mark.parametrize('center,sign', [(180, 1), (460, -1)])
def test_off_axis_width_uses_axial_depth_and_range_is_radial(center, sign):
    settings = DetectorSettings()
    blobs, _ = find_black_blobs(scene(center=center), camera(), settings)
    item = measure_blob(blobs[0], 1.5, 'depth', settings)
    assert item.width_m == pytest.approx(0.27, abs=1e-6)
    assert sign * item.bearing > 0
    assert item.distance_m == pytest.approx(1.5 / math.cos(item.bearing))


@pytest.mark.parametrize('color', [(0, 0, 65), (0, 0, 220), (180, 180, 180)])
def test_saturated_dark_colors_and_bright_objects_are_rejected(color):
    assert find_black_blobs(scene(color=color), camera(), DetectorSettings())[0] == []


def test_depth_samples_only_the_contour_and_never_assumes_a_known_width():
    settings = DetectorSettings()
    blob = find_black_blobs(scene(), camera(), settings)[0][0]
    depth = np.full((480, 640), 4., np.float32)
    depth[180:340, 275:365] = 1.5
    depth[200:220, 290:310] = np.nan
    assert depth_for_blob(blob, depth, settings) == 1.5
    assert math.isnan(depth_for_blob(blob, np.full_like(depth, np.nan), settings))
    assert measure_blob(blob, math.nan, 'depth', settings) is None


def test_clipped_blob_cannot_pass_width_filter():
    settings = DetectorSettings()
    blob = find_black_blobs(scene(center=20), camera(), settings)[0][0]
    assert blob.clipped
    assert measure_blob(blob, 1.5, 'depth', settings) is None


def test_color_speckles_too_narrow_at_maximum_range_are_ignored():
    assert not find_black_blobs(scene(width=12), camera(), DetectorSettings())[0]


def test_lidar_projects_into_contour_and_rejects_wrong_height_and_background():
    settings = DetectorSettings()
    blob = find_black_blobs(scene(), camera(), settings)[0][0]
    # Five foreground hits; far background also projects into the silhouette.
    foreground = np.column_stack((np.linspace(-0.1, 0.1, 5), np.zeros(5), np.full(5, 1.5)))
    background = np.column_stack((np.linspace(-0.1, 0.1, 8), np.zeros(8), np.full(8, 4.0)))
    wrong_height = foreground.copy()
    wrong_height[:, 1] = -1.0
    ranges = lidar_for_blobs([blob], np.vstack((foreground, background, wrong_height)),
                             camera(), settings)
    assert ranges == [1.5]
    assert math.isnan(lidar_for_blobs([blob], wrong_height, camera(), settings)[0])
    assert math.isnan(lidar_for_blobs([blob], foreground[:2], camera(), settings)[0])


def test_lidar_extrinsics_include_translation_and_optical_axis_rotation():
    scan = SimpleNamespace(ranges=[2., np.inf, 0., np.nan], angle_min=0.,
                           angle_increment=0.1, range_min=0.15, range_max=5.)
    rotation = np.array([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])
    points = scan_points_in_camera(scan, rotation, [0.1, 0.2, -0.3])
    np.testing.assert_allclose(points, [[0.1, 0.2, 1.7]])


def test_rectified_camera_uses_projection_matrix_and_scales_it():
    info = SimpleNamespace(width=640, height=480, k=[500., 0, 320, 0, 500, 240, 0, 0, 1],
                           p=[400., 0, 320, 0, 0, 400, 240, 0, 0, 0, 1, 0],
                           d=[0.] * 5, distortion_model='plumb_bob')
    assert CameraModel.from_info(info, 320, 240, True).matrix[0, 0] == 200
    assert CameraModel.from_info(info, 320, 240, False).matrix[0, 0] == 250
