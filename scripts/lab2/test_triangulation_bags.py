"""Test every triangulation method on the recorded bags against tape/Zamo ground truth.

    python scripts/lab2/test_triangulation_bags.py [BAG_FOLDER]   (laptop, default: runs/bags)

Needs: pip install rosbags opencv-python numpy, and the l2s2_localisation package
(team branch + rover_additions) on PYTHONPATH. No ROS installation needed.

For each test moment (rover standing still at a measured spot):
  * every tag detection -> range + bearing in the robot frame (geometry.observe,
    same mount model and tag size 0.14 m as on the rover; pan = commanded pan)
  * "scan" fix: per tag, the median over the frames nearest the image centre,
    then each method once
  * "per image" fixes: every single image with 2+ mapped tags, each method
"""
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

from l2s2_localisation.geometry import MountModel, observe
from l2s2_localisation.live_triangulation import solve as live_solve
from l2s2_localisation.triangulation import Field, Landmark, localize
from l2s2_localisation.triangulation_advanced import NoiseModel, localize_advanced, localize_robust

SIZE = 0.14
FIELD = Field(4.5, 3.0, 0.7)
NOISE = NoiseModel(sigma_pan_deg=2.0)
PAN_SIGN = -1.0
STEADY_S = 0.8            # pan unchanged this long before the image -> camera settled
MAP = {0: (-0.55, 0.0), 1: (-1.5, -3.0), 2: (-1.5, 3.0), 3: (1.55, -3.0), 4: (1.55, 3.0),
       5: (4.5, 0.0)}
BAGS = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[2] / 'runs' / 'bags'
TESTS = [  # bag, name, ground truth (x, y), which still phase
    ('center_sweep2_221736', 'centre spot (stepped sweep)', (0.0, 0.0), 'all'),
    ('drive_straight_223727', 'end of straight drive', (2.06, -0.29), 'end'),
    ('drive_arc2_225442', 'start of 2nd drive', (-3.25, 0.0), 'start'),
    ('drive_arc_sweep_225831', 'end of curve (continuous sweep)', (-0.79, 0.81), 'end'),
]


def read_bag(path):
    store = get_typestore(Stores.ROS2_HUMBLE)
    data = {'pan': [], 'odom': [], 'det': [], 'info': None}
    topics = {'/ugv/pan_tilt_commanded', '/odom', '/marker_detector/detections', '/camera_info'}
    with AnyReader([path], default_typestore=store) as reader:
        connections = [c for c in reader.connections if c.topic in topics]
        for connection, _, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, connection.msgtype)
            if connection.topic == '/camera_info':
                data['info'] = data['info'] or msg
                continue
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 \
                if hasattr(msg, 'header') else None
            if connection.topic == '/ugv/pan_tilt_commanded':
                names = list(msg.name)
                data['pan'].append((stamp, PAN_SIGN * msg.position[names.index('pt_base_link_to_pt_link1')]))
            elif connection.topic == '/odom':
                p = msg.pose.pose.position
                data['odom'].append((stamp, p.x, p.y))
            else:
                data['det'].append(json.loads(msg.data))
    for key in ('pan', 'odom'):
        data[key].sort()
    return data


def pan_at(pans, stamp):
    """Commanded pan at the image time and how long it had been unchanged."""
    before = [(t, p) for t, p in pans if t <= stamp]
    if not before:
        return None, 0.0
    pan = before[-1][1]
    since = before[0][0]
    for t, p in reversed(before):
        if abs(p - pan) > math.radians(0.5):
            break
        since = t
    return pan, stamp - since


def still_window(odom, phase):
    """(t_from, t_to) of the standing-still phase from odometry (1 cm tolerance)."""
    if phase == 'all':
        return -math.inf, math.inf
    xy = np.array([(x, y) for _, x, y in odom])
    times = [t for t, _, _ in odom]
    if phase == 'start':
        moved = np.linalg.norm(xy - xy[0], axis=1) > 0.01
        return -math.inf, times[int(np.argmax(moved))] if moved.any() else math.inf
    moved = np.linalg.norm(xy - xy[-1], axis=1) > 0.01
    last = int(np.nonzero(moved)[0][-1]) if moved.any() else 0
    return times[last] + 0.5, math.inf


def observations(data, phase, steady_only):
    info = data['info']
    K = np.array(info.p, float).reshape(3, 4)[:, :3]          # rectified image
    camera = SimpleNamespace(matrix_for=lambda w, h: K * np.array([[w / info.width], [h / info.height], [1]]),
                             D=np.zeros(5))
    t_from, t_to = still_window(data['odom'], phase)
    frames = []
    for det in data['det']:
        stamp = float(det['stamp'])
        if not t_from <= stamp <= t_to:
            continue
        pan, steady = pan_at(data['pan'], stamp)
        if pan is None or (steady_only and steady < STEADY_S):
            continue
        frame = []
        for item in det.get('detections', []):
            if item['id'] not in MAP:
                continue
            try:
                frame.append(observe(item['family'], item['id'], item['corners'], SIZE, camera,
                                     (det['width'], det['height']), MountModel(), pan, 0.0, stamp))
            except ValueError:
                continue
        if frame:
            frames.append(frame)
    return frames


def landmark(o):
    return Landmark(f'ID{o.marker_id}', MAP[o.marker_id], o.range, o.bearing, o.side_px)


def scan_landmarks(frames):
    by_tag = {}
    for frame in frames:
        for o in frame:
            by_tag.setdefault(o.marker_id, []).append(o)
    out = []
    for tag, obs in sorted(by_tag.items()):
        obs.sort(key=lambda o: o.centre_offset)
        near = obs[:max(3, len(obs) // 3)]
        ref = near[0].bearing
        bearing = ref + float(np.median([math.remainder(o.bearing - ref, math.tau) for o in near]))
        out.append(Landmark(f'ID{tag}', MAP[tag], float(np.median([o.range for o in near])),
                            bearing, float(np.median([o.side_px for o in near]))))
    return out


METHODS = {
    'line (yours)': lambda m: tuple(localize(m, FIELD).position),
    'trig (advanced)': lambda m: (lambda f: (f.x, f.y))(localize_advanced(m, NOISE, FIELD)),
    'trig + rejection': lambda m: (lambda f: (f[0].x, f[0].y))(localize_robust(m, NOISE, FIELD)),
    'live (team)': lambda m: tuple(live_solve(m, FIELD)['position_m'][:2]),
}


def run(m, truth):
    row = {}
    for name, method in METHODS.items():
        try:
            row[name] = math.dist(method(m), truth)
        except (ValueError, np.linalg.LinAlgError) as error:
            row[name] = None
    return row


def main():
    results = []
    for bag, name, truth, phase in TESTS:
        data = read_bag(BAGS / bag)
        steady = phase != 'end' or 'sweep' not in name
        frames = observations(data, phase, steady_only=steady)
        tags = sorted({o.marker_id for f in frames for o in f})
        print(f'\n== {name}  truth {truth}  ({bag}; {len(frames)} frames with tags {tags}'
              f'{"" if steady else "; camera moving: commanded pan used"})')
        marks = scan_landmarks(frames)
        for m in marks:
            print(f'   {m.label}: range {m.range:.2f} m  bearing {math.degrees(m.bearing):+6.1f} deg'
                  f'  map distance from truth {math.dist(m.position, truth):.2f} m')
        scan = run(marks, truth) if len(marks) >= 2 else {}
        per_image = {k: [] for k in METHODS}
        multi = [f for f in frames if len({o.marker_id for o in f}) >= 2]
        for frame in multi:
            for k, v in run([landmark(o) for o in frame], truth).items():
                per_image[k].append(v)
        print(f'   {"method":18} {"scan fix err":>13} {"per image: n ok":>16} {"median err":>11}')
        for k in METHODS:
            ok = [v for v in per_image[k] if v is not None]
            s = scan.get(k)
            print(f'   {k:18} {("%.2f m" % s) if s is not None else "fails":>13} '
                  f'{len(ok):>8}/{len(multi):<7} {("%.2f m" % np.median(ok)) if ok else "-":>11}')
        results.append({'test': name, 'bag': bag, 'truth': truth, 'tags': tags,
                        'scan_error_m': scan, 'per_image_frames': len(multi),
                        'per_image_median_m': {k: (float(np.median([v for v in vs if v is not None]))
                                                   if any(v is not None for v in vs) else None)
                                               for k, vs in per_image.items()}})
    out = BAGS.parent / 'triangulation_results.json'
    out.write_text(json.dumps(results, indent=2))
    print(f'\nsaved {out}')


if __name__ == '__main__':
    sys.exit(main())
