"""Shared field geometry, planar alignment and optional occupancy-map export."""

from array import array
import math
from pathlib import Path

import yaml

START_KEYS = ('start_x_m', 'start_y_m', 'start_heading_deg')
DEFAULTS = {
    'start_x_m': -3.2, 'start_y_m': 0.0, 'start_heading_deg': 0.0,
    'initialize_on_first_odom': True, 'publish_tf': True,
    'map_frame': 'map', 'odom_frame': 'odom', 'odom_timeout_s': 1.0,
    'field_length_m': 9.0, 'field_width_m': 6.0, 'border_m': 0.7,
    'line_width_m': 0.05, 'center_circle_radius_m': 0.75,
    'penalty_mark_distance_m': 1.3, 'penalty_mark_size_m': 0.1,
    'penalty_area_depth_m': 1.65, 'penalty_area_width_m': 4.0,
    'goal_area_depth_m': 0.6, 'goal_area_width_m': 2.2,
    'goal_opening_width_m': 1.5, 'goal_depth_m': 0.5,
    'goal_height_m': 0.8, 'goal_post_diameter_m': 0.1,
    'map_resolution_m': 0.01, 'map_goal_obstacles': False,
}


def validate(config):
    """Reject invalid dimensions and poses before updating a display or transform."""
    for name, default in DEFAULTS.items():
        value = config[name]
        if isinstance(default, bool):
            if not isinstance(value, bool):
                raise ValueError(f'{name} must be true or false')
        elif isinstance(default, str):
            if not isinstance(value, str) or not value or value.startswith('/'):
                raise ValueError(f'{name} must be a nonempty ROS frame without a leading slash')
        elif isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'{name} must be a finite number')
        elif name not in START_KEYS and value <= 0:
            raise ValueError(f'{name} must be positive')
    if config['map_frame'] == config['odom_frame']:
        raise ValueError('map_frame and odom_frame must differ')
    for prefix in ('penalty_area', 'goal_area'):
        if config[prefix + '_width_m'] > config['field_width_m']:
            raise ValueError(f'{prefix} is wider than the field')
        if config[prefix + '_depth_m'] >= config['field_length_m'] / 2:
            raise ValueError(f'{prefix} is too deep for the field')
    if config['penalty_mark_distance_m'] >= config['field_length_m'] / 2:
        raise ValueError('penalty mark must lie in its own half')
    width, height, _ = map_shape(config)
    if width < 1 or height < 1:
        raise ValueError('map resolution is larger than the field')
    if width * height > 4_000_000:
        raise ValueError('map resolution produces more than 4 million cells')


def load_config(path):
    data = yaml.safe_load(Path(path).read_text()) or {}
    if not isinstance(data, dict) or not isinstance(data.get('football_field'), dict):
        raise ValueError('Expected football_field: ros__parameters: in field configuration')
    values = data.get('football_field', {}).get('ros__parameters', {})
    if not isinstance(values, dict):
        raise ValueError('Field ros__parameters must be a mapping')
    unknown = set(values) - set(DEFAULTS)
    if unknown:
        raise ValueError(f'Unknown field setting(s): {", ".join(sorted(unknown))}')
    config = {**DEFAULTS, **values}
    validate(config)
    for key, default in DEFAULTS.items():
        if isinstance(default, float):
            config[key] = float(config[key])
    return config


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def alignment(start, odom):
    """map <- odom, making the supplied odometry pose equal the desired start."""
    yaw = wrap(math.radians(start[2]) - odom[2])
    c, s = math.cos(yaw), math.sin(yaw)
    return (start[0] - c * odom[0] + s * odom[1],
            start[1] - s * odom[0] - c * odom[1], yaw)


def transform_pose(transform, pose):
    x, y, yaw = transform
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * pose[0] - s * pose[1],
            y + s * pose[0] + c * pose[1], wrap(yaw + pose[2]))


def field_layout(config):
    """Render primitives in metres, used unchanged by both browser views."""
    c = config
    hx, hy = c['field_length_m'] / 2, c['field_width_m'] / 2
    lines = []

    def line(points, color='#ffffff', width=None):
        lines.append({'points': [[x, y, z] for x, y, z in points], 'color': color,
                      'width_m': width or c['line_width_m']})

    def rectangle(x0, x1, y0, y1, color='#ffffff'):
        line([(x0, y0, .006), (x1, y0, .006), (x1, y1, .006),
              (x0, y1, .006), (x0, y0, .006)], color)

    rectangle(-hx, hx, -hy, hy)
    line([(0, -hy, .006), (0, hy, .006)])
    r = c['center_circle_radius_m']
    line([(r * math.cos(i * math.tau / 96), r * math.sin(i * math.tau / 96), .006)
          for i in range(97)])
    mark = c['penalty_mark_size_m'] / 2
    line([(-mark, 0, .007), (mark, 0, .007)])
    for sign in (-1, 1):
        goal_x = sign * hx
        for prefix in ('penalty_area', 'goal_area'):
            inner_x = sign * (hx - c[prefix + '_depth_m'])
            half_width = c[prefix + '_width_m'] / 2
            line([(goal_x, -half_width, .006), (inner_x, -half_width, .006),
                  (inner_x, half_width, .006), (goal_x, half_width, .006)])
        spot = sign * (hx - c['penalty_mark_distance_m'])
        line([(spot - mark, 0, .007), (spot + mark, 0, .007)])
        line([(spot, -mark, .007), (spot, mark, .007)])
        # Post centres leave the configured clear opening between their inner edges.
        post_y = (c['goal_opening_width_m'] + c['goal_post_diameter_m']) / 2
        back_x, z = goal_x + sign * c['goal_depth_m'], c['goal_height_m']
        line([(goal_x, -post_y, 0), (goal_x, -post_y, z),
              (goal_x, post_y, z), (goal_x, post_y, 0)], '#f2f2f2',
             c['goal_post_diameter_m'])
        line([(goal_x, -post_y, .008), (back_x, -post_y, .008),
              (back_x, post_y, .008), (goal_x, post_y, .008)], '#b9cfbc', .025)
    b = c['border_m']
    return {'frame_id': c['map_frame'], 'length_m': 2 * hx, 'width_m': 2 * hy,
            'border_m': b, 'ground_color': '#28753c',
            'ground': [[-hx-b, -hy-b, 0], [hx+b, -hy-b, 0],
                       [hx+b, hy+b, 0], [-hx-b, hy+b, 0]], 'lines': lines}


def map_shape(config):
    resolution = config['map_resolution_m']
    width = round((config['field_length_m'] + 2 * config['border_m']) / resolution)
    height = round((config['field_width_m'] + 2 * config['border_m']) / resolution)
    return width, height, (-width * resolution / 2, -height * resolution / 2)


def occupancy(config):
    """Bottom-up ROS occupancy: free carpet; optional goal footprints, no walls/paint."""
    width, height, origin = map_shape(config)
    data = array('b', [0]) * (width * height)
    if config['map_goal_obstacles']:
        res = config['map_resolution_m']
        radius = config['goal_post_diameter_m'] / 2
        post_y = (config['goal_opening_width_m'] + 2 * radius) / 2
        for sign in (-1, 1):
            gx = sign * config['field_length_m'] / 2
            back = gx + sign * config['goal_depth_m']
            for iy in range(height):
                y = origin[1] + (iy + .5) * res
                for ix in range(max(0, int((min(gx, back)-radius-origin[0])/res)),
                                min(width, math.ceil((max(gx, back)+radius-origin[0])/res))):
                    x = origin[0] + (ix + .5) * res
                    posts = any((x-gx)**2 + (y-py)**2 <= radius**2
                                for py in (-post_y, post_y))
                    net = (min(gx, back) <= x <= max(gx, back)
                           and abs(abs(y) - post_y) <= .025)
                    net |= abs(x-back) <= .025 and abs(y) <= post_y
                    if posts or net:
                        data[iy * width + ix] = 100
    return data


def export_map(config, directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    width, height, origin = map_shape(config)
    data = occupancy(config)
    # PGM is top-down, ROS grid is bottom-up. Painted lines remain free.
    pixels = b''.join(bytes(0 if v else 254 for v in data[row*width:(row+1)*width])
                      for row in range(height-1, -1, -1))
    (directory / 'robocup_field.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode() + pixels)
    metadata = {'image': 'robocup_field.pgm', 'mode': 'trinary',
                'resolution': config['map_resolution_m'], 'origin': [*origin, 0.0],
                'negate': 0, 'occupied_thresh': .65, 'free_thresh': .25}
    (directory / 'robocup_field.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False))
