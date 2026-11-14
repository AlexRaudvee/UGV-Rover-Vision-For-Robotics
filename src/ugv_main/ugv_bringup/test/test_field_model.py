"""Coordinate and map checks for the standard field, independent of ROS."""

import math
from pathlib import Path

import pytest

from ugv_bringup.field_model import (DEFAULTS, alignment, export_map, field_layout,
                                    load_config, map_shape, occupancy, transform_pose,
                                    validate)


@pytest.mark.parametrize('odom,start', [
    ((0, 0, 0), (-3.2, 0, 0)),
    ((1.2, -2, .7), (-3.2, 0, 0)),
    ((-2, .3, -3.1), (3.2, 0, 180)),
    ((8, -3, 2.5), (.5, -1, 90)),
])
def test_nonzero_odometry_is_aligned_to_desired_start(odom, start):
    mapped = transform_pose(alignment(start, odom), odom)
    assert mapped[:2] == pytest.approx(start[:2])
    assert math.cos(mapped[2]) == pytest.approx(math.cos(math.radians(start[2])))
    assert math.sin(mapped[2]) == pytest.approx(math.sin(math.radians(start[2])))


def test_forward_travel_follows_selected_start_heading():
    odom = (1, 2, .7)
    transform = alignment((-3.2, 0, 90), odom)
    moved = (odom[0] + math.cos(.7), odom[1] + math.sin(.7), .7)
    assert transform_pose(transform, moved)[:2] == pytest.approx((-3.2, 1))


def test_standard_markings_and_carpet_bounds():
    layout = field_layout(DEFAULTS)
    assert layout['ground'][0] == pytest.approx([-5.2, -3.7, 0])
    points = [p for line in layout['lines'] for p in line['points']]
    assert any(p[:2] == pytest.approx([-3.2, -.05]) for p in points)
    assert any(p[:2] == pytest.approx([3.2, .05]) for p in points)
    assert any(p[:2] == pytest.approx([.75, 0]) for p in points)


def test_default_map_has_no_invented_obstacles_or_painted_walls(tmp_path):
    width, height, origin = map_shape(DEFAULTS)
    assert (width, height) == (1040, 740)
    assert origin == pytest.approx((-5.2, -3.7))
    data = occupancy(DEFAULTS)
    assert len(data) == width * height and set(data) == {0}
    export_map(DEFAULTS, tmp_path)
    header, pixels = (tmp_path / 'robocup_field.pgm').read_bytes().split(b'255\n', 1)
    assert header == b'P5\n1040 740\n'
    assert len(pixels) == width * height and set(pixels) == {254}


def test_optional_goals_have_clear_opening_and_no_perimeter_wall():
    config = {**DEFAULTS, 'map_goal_obstacles': True}
    data = occupancy(config)
    width, height, origin = map_shape(config)
    def cell(x, y):
        ix, iy = int((x-origin[0])/.01), int((y-origin[1])/.01)
        return data[iy*width+ix]
    assert cell(4.5, .8) == 100
    assert cell(4.5, 0) == 0
    assert cell(0, 3.65) == 0
    assert 0 < sum(v == 100 for v in data) < .01 * width * height


@pytest.mark.parametrize('key,value', [('start_x_m', float('nan')),
                                      ('start_heading_deg', float('inf')),
                                      ('map_resolution_m', 0), ('field_width_m', -1)])
def test_invalid_pose_or_dimensions_are_rejected(key, value):
    with pytest.raises(ValueError):
        validate({**DEFAULTS, key: value})


def test_integer_pose_entries_are_normalized_to_ros_doubles(tmp_path):
    config_file = tmp_path / 'field.yaml'
    config_file.write_text('football_field:\n  ros__parameters:\n    start_x_m: -3\n')
    assert type(load_config(config_file)['start_x_m']) is float
