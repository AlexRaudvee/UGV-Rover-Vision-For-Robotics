"""Marker map + triangulation for pan-tilt scan results (no ROS imports).

Offline:
    python3 -m l2s2_localisation.localize runs/marker_scans/scan_*.json \
        --map config/marker_map.yaml [--no-refine] [--no-bearings]
"""

import argparse
import json
import math
import sys

from l2s1_marker_detection import aruco_compat as compat
from .triangulation import Field, Landmark, localize
from .triangulation_advanced import localize_advanced, NoiseModel


def load_map(path):
    """{(family, id): (x, y)} from a YAML file with 'markers: {FAMILY:ID: [x, y]}'."""
    import yaml
    with open(path) as stream:
        data = yaml.safe_load(stream) or {}
    world = {}
    for key, value in (data.get('markers') or {}).items():
        family, _, marker_id = str(key).rpartition(':')
        if not family or len(value) < 2:
            raise ValueError(f'Map entry {key!r}: expected "FAMILY:ID: [x, y]"')
        world[(compat.normalize_family(family), int(marker_id))] = (float(value[0]),
                                                                   float(value[1]))
    field_data = data.get('field') or {}
    return world, Field(**{k: float(v) for k, v in field_data.items()})


def landmarks_from_scan(scan_markers, world, use_bearings=True):
    """Match scanned markers (scanner summary entries) with known field positions."""
    landmarks, unknown = [], []
    for entry in scan_markers:
        key = (entry['family'], int(entry['id']))
        if key not in world:
            unknown.append(f'{key[0]}:{key[1]}')
            continue
        bearing = math.radians(entry['bearing_deg']) if use_bearings else None
        landmarks.append(Landmark(f'{key[0]}:{key[1]}', world[key], entry['range_m'], bearing,
                                  entry.get('side_px')))
    return landmarks, unknown


def localize_scan(scan_markers, world, field_model, use_refine=True, use_bearings=True,
                  noise=None):
    """Result dict for a scan, or an explanation when fewer than 2 markers are mapped.

    The top level holds the two-circle / least-squares fix (ranges). When bearings
    are available, 'advanced' holds the range + bearing fix for comparison.
    """
    landmarks, unknown = landmarks_from_scan(scan_markers, world, use_bearings)
    result = {'markers_used': [m.label for m in landmarks], 'markers_not_in_map': unknown}
    if len(landmarks) < 2:
        result['error'] = f'{len(landmarks)} mapped marker(s) seen; need at least 2'
        return result
    result.update(localize(landmarks, field_model, use_refine).as_dict())
    if use_bearings:
        try:
            result['advanced'] = localize_advanced(landmarks, noise or NoiseModel(),
                                                   field_model).as_dict()
        except ValueError as error:
            result['advanced'] = {'error': str(error)}
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('scans', nargs='+', help='scan_*.json files from the marker scanner')
    parser.add_argument('--map', required=True, help='marker map YAML')
    parser.add_argument('--no-refine', action='store_true',
                        help='linear least squares only (compare with the refined fix)')
    parser.add_argument('--no-bearings', action='store_true',
                        help='ignore bearings (shows the two-solution ambiguity)')
    parser.add_argument('--sigma-pan-deg', type=float, default=1.0,
                        help='measured pan servo error for the advanced method')
    parser.add_argument('--sigma-px', type=float, default=0.5)
    args = parser.parse_args(argv)
    world, field_model = load_map(args.map)
    noise = NoiseModel(sigma_px=args.sigma_px, sigma_pan_deg=args.sigma_pan_deg)
    for path in args.scans:
        with open(path) as stream:
            scan = json.load(stream)
        result = localize_scan(scan['markers'], world, field_model, not args.no_refine,
                               not args.no_bearings, noise)
        print(f'{path}:')
        print(f'  basic    (ranges):           x={result.get("x_m")} y={result.get("y_m")} '
              f'{result.get("error", "")}')
        advanced = result.get('advanced', {})
        if 'x_m' in advanced:
            print(f'  advanced (range + bearing):  x={advanced["x_m"]} y={advanced["y_m"]} '
                  f'heading={advanced["heading_deg"]} deg  '
                  f'+/- {advanced["sigma_x_m"]}, {advanced["sigma_y_m"]} m')
        print(f'  {json.dumps(result)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
