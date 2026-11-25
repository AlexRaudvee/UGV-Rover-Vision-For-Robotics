"""Field-map behavior without ROS, camera drivers, planning or motor commands."""

import math

import numpy as np
import pytest

from l2s3_curling.obstacle_map import GridSettings, ObstacleMap, Stand


def cell(model, grid, x, y):
    ix, iy = np.floor((np.array([x, y])-model.origin)/model.settings.resolution).astype(int)
    return grid[iy, ix]


def test_empty_detections_leave_unseen_field_unknown():
    model = ObstacleMap()
    model.update([], [], 10.)
    grid, inflated = model.render((0, 0, 0), 10.)
    assert cell(model, grid, 0, 0) == -1
    assert cell(model, inflated, 0, 0) == -1


def test_real_free_patches_clear_a_removed_obstacle_but_missing_data_does_not():
    model = ObstacleMap()
    model.update([], [[1.02, .02]], 10.)
    assert cell(model, model.render((0, 0, 0), 10.)[0], 1.02, .02) == 100
    model.update([], [], 10.5)
    assert cell(model, model.render((0, 0, 0), 10.5)[0], 1.02, .02) == 100
    model.update([[1.02, .02]], [], 10.7)
    assert cell(model, model.render((0, 0, 0), 10.7)[0], 1.02, .02) == 0
    assert cell(model, model.render((0, 0, 0), 12.)[0], 1.02, .02) == -1


def test_expired_obstacles_become_unknown_not_free():
    model = ObstacleMap()
    model.update([], [[1.02, .02]], 10.)
    assert cell(model, model.render((0, 0, 0), 14.)[0], 1.02, .02) == -1


def test_ekf_correction_reprojects_live_evidence_without_moving_known_stands():
    model = ObstacleMap(stands=[Stand('tag:0', 0., 0.)])
    model.update([], [[1.025, .025]], 10.)
    initial = model.render((0, 0, 0), 10.)[0]
    corrected = model.render((0., 1., math.pi/2), 10.)[0]
    assert cell(model, initial, 1.025, .025) == 100
    assert cell(model, corrected, -.025, 2.025) == 100
    assert cell(model, corrected, 1.025, .025) == -1
    assert cell(model, initial, 0, 0) == cell(model, corrected, 0, 0) == 100


def test_same_image_obstacle_hits_override_floor_and_inflate_for_rover_footprint():
    model = ObstacleMap()
    model.update([[1.025, .025]], [[1.025, .025]], 10.)
    grid, inflated = model.render((0, 0, 0), 10.)
    assert cell(model, grid, 1.025, .025) == 100
    assert cell(model, grid, 1.175, .025) == -1
    assert cell(model, inflated, 1.175, .025) == 100
    assert cell(model, inflated, 2., 0.) == -1


def test_bad_tf_or_unusable_input_never_displays_fresh_free_space():
    model = ObstacleMap()
    model.update([[1.025, .025]], [], 10.)
    assert cell(model, model.render(None, 10.)[0], 1.025, .025) == -1
    assert cell(model, model.render((0, 0, 0), 10., allow_free=False)[0], 1.025, .025) == -1


def test_all_obstacle_faces_are_rasterized_without_assuming_rear_extent():
    model = ObstacleMap()
    model.add_face([[1., -.2], [1., .2]], 10., .04)
    grid = model.render((0, 0, 0), 10.)[0]
    assert cell(model, grid, 1.025, .025) == 100
    assert cell(model, grid, 1.5, .025) == -1


def test_fixed_square_is_placed_behind_front_independent_of_apparent_width():
    model = ObstacleMap()
    model.add_square([[1., -.3], [1., .3]], [0., 0.], 10., size=.3)
    grid, _ = model.render((0, 0, 0), 10., include_static=False)
    assert cell(model, grid, 1.125, .025) == 100
    assert cell(model, grid, .825, .025) == -1
    assert cell(model, grid, 1.125, .225) == -1
    assert cell(model, grid, 1.425, .025) == -1


def test_stand_footprint_survives_sensor_clearing_and_map_reset():
    model = ObstacleMap(stands=[Stand('tag:0', 0., 0., width=.4, depth=.2)])
    model.update([[.025, .025]], [[1.025, .025]], 10.)
    model.clear()
    grid = model.render((0, 0, 0), 10.)[0]
    assert cell(model, grid, .025, .025) == 100
    assert cell(model, grid, 1.025, .025) == -1


def test_detection_overlay_excludes_configured_stands_and_boundary():
    model = ObstacleMap(stands=[Stand('tag:0', 0., 0.)])
    model.add_face([[1., -.1], [1., .1]], 10.)
    grid, _ = model.render((0, 0, 0), 10., allow_free=False, include_static=False)
    assert cell(model, grid, 0., 0.) == -1
    assert cell(model, grid, -4.49, 0.) == -1
    assert cell(model, grid, 1.025, .025) == 100


@pytest.mark.parametrize('kwargs', [dict(resolution=0), dict(half_width=float('nan')),
                                  dict(free_memory_s=-1), dict(resolution=.0001)])
def test_bad_grid_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        GridSettings(**kwargs)


def test_live_faces_require_repeated_frames_and_replace_old_footprints():
    model = ObstacleMap()
    camera = np.zeros(2)
    face = np.array([[1., -.15], [1., .15]])
    for i in range(3):
        model.update_faces([(face, 0.)], camera, 10.+i*.1)
        assert not model.cells
    model.update_faces([(face, 0.)], camera, 10.3)
    assert model.cells
    for i in range(5):
        model.update_faces([(face+[.2, 0.], 0.)], camera, 10.4+i*.1)
    assert all(x * model.settings.resolution >= 1.15 for x, _ in model.cells)
    model.update_faces([], camera, 13.)
    assert not model.cells
