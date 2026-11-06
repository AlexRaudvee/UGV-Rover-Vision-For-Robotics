#!/usr/bin/env python3
"""Export every frame of an eval bag as a 6-panel image plus a CSV of per-frame flags.

Usage (ROS sourced):
    python3 scripts/export_eval_frames.py ~/bags/eval_run3 [out_dir]

Panels: depth detector | lidar detector / depth image | status text /
depth histogram | lidar histogram. Flags in frames.csv point at frames worth a look.
"""

import bisect
import csv
import json
import os
import sys

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

IMAGES = {
    'depth_dbg': '/object_detection/debug/compressed',
    'lidar_dbg': '/obstacle_detection/debug/compressed',
    'depth_img': '/object_detection/depth/compressed',
    'hist_depth': '/polar_hist/object/histogram/compressed',
    'hist_lidar': '/polar_hist/obstacle/histogram/compressed',
}
JSONS = {'depth': '/object_detection/detections', 'lidar': '/obstacle_detection/detections'}
W, H = 640, 360
LIFTED_AFTER_S = 45.0   # in avoidance_cleaned_v2 the robot is picked up after ~45 s


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    storage = 'mcap' if any(f.endswith('.mcap') for f in os.listdir(path)) else 'sqlite3'
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    data = {}
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic in IMAGES.values():
            data.setdefault(topic, []).append((stamp, deserialize_message(raw, CompressedImage)))
        elif topic in JSONS.values():
            data.setdefault(topic, []).append(
                (stamp, json.loads(deserialize_message(raw, String).data)))
    return data


def nearest(series, stamp, max_dt=0.4e9):
    if not series:
        return None
    keys = [s for s, _ in series]
    i = bisect.bisect_left(keys, stamp)
    best = min((j for j in (i - 1, i) if 0 <= j < len(series)),
               key=lambda j: abs(keys[j] - stamp))
    return series[best][1] if abs(keys[best] - stamp) <= max_dt else None


def decode(msg):
    if msg is None:
        img = np.zeros((H, W, 3), np.uint8)
        cv2.putText(img, 'no data', (250, 180), cv2.FONT_HERSHEY_SIMPLEX, 1, (90, 90, 90), 2)
        return img
    img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
    return cv2.resize(img, (W, H))


def summary(det):
    """Short per-detector description and its accepted boxes."""
    if det is None:
        return 'no message', []
    if not det.get('fresh'):
        return 'not fresh: ' + det.get('status', '')[:60], []
    boxes = [(d['bearing'], d['distance_m'], d['width_m']) for d in det['detections']]
    return det.get('status', ''), boxes


def flags_for(t, depth, lidar, depth_boxes, lidar_boxes, band):
    flags = []
    if t > LIFTED_AFTER_S:
        flags.append('robot_lifted')
    if depth and (depth.get('blob_count') or 0) >= 8:
        flags.append('clutter_%d_blobs' % depth['blob_count'])
    if depth and depth.get('blob_count') and not depth_boxes:
        flags.append('depth_blobs_none_accepted')
    for b, r, w in depth_boxes + lidar_boxes:
        if band and (w - band[0] < 0.01 or band[1] - w < 0.01):
            flags.append('width_at_band_edge_%.0fcm' % (w * 100))
            break
    for b, r, _ in depth_boxes:   # same object seen by both detectors but at different range
        for lb, lr, _ in lidar_boxes:
            if abs(b - lb) < 0.05 and abs(r - lr) > 0.5:
                flags.append('depth_lidar_disagree_%.1fm_vs_%.1fm' % (r, lr))
    return flags


def main():
    bag = os.path.expanduser(sys.argv[1]).rstrip('/')
    out = os.path.expanduser(sys.argv[2]) if len(sys.argv) > 2 else bag + '_frames'
    os.makedirs(out, exist_ok=True)
    data = read_bag(bag)
    frames = data.get(IMAGES['depth_dbg'], [])
    if not frames:
        sys.exit('No depth debug images in ' + bag)
    t0 = frames[0][0]
    rows, prev_n = [], None
    for idx, (stamp, dbg) in enumerate(frames):
        t = (stamp - t0) / 1e9
        depth = nearest(data.get(JSONS['depth'], []), stamp)
        lidar = nearest(data.get(JSONS['lidar'], []), stamp)
        d_status, d_boxes = summary(depth)
        l_status, l_boxes = summary(lidar)
        band = (depth or {}).get('width_band_m')
        flags = flags_for(t, depth, lidar, d_boxes, l_boxes, band)
        if prev_n is not None and prev_n > 0 and not d_boxes and depth and depth.get('fresh'):
            flags.append('depth_lost_box_since_prev_frame')
        prev_n = len(d_boxes)

        info = np.full((H, W, 3), 30, np.uint8)
        lines = ['frame %d   t = %.1f s' % (idx, t), '', 'DEPTH: ' + d_status[:52]]
        lines += ['  %+5.1f deg  %.2f m  %.1f cm' % (np.degrees(b), r, w * 100)
                  for b, r, w in d_boxes[:4]]
        lines += ['', 'LIDAR: ' + l_status[:52]]
        lines += ['  %+5.1f deg  %.2f m  %.1f cm' % (np.degrees(b), r, w * 100)
                  for b, r, w in l_boxes[:4]]
        lines += ['', 'FLAGS: ' + (', '.join(flags) if flags else '-')]
        y = 30
        for line in lines:
            cv2.putText(info, line[:62], (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 200, 255) if line.startswith('FLAGS') and flags else (230, 230, 230), 1)
            y += 24
        panels = [decode(dbg), decode(nearest(data.get(IMAGES['lidar_dbg'], []), stamp)),
                  decode(nearest(data.get(IMAGES['depth_img'], []), stamp)), info,
                  decode(nearest(data.get(IMAGES['hist_depth'], []), stamp)),
                  decode(nearest(data.get(IMAGES['hist_lidar'], []), stamp))]
        grid = np.vstack([np.hstack(panels[i:i + 2]) for i in (0, 2, 4)])
        name = 'f%03d_t%05.1f.jpg' % (idx, t)
        cv2.imwrite(os.path.join(out, name), grid, [cv2.IMWRITE_JPEG_QUALITY, 85])
        rows.append({'file': name, 'frame': idx, 't_s': round(t, 2),
                     'depth_accepted': len(d_boxes), 'lidar_accepted': len(l_boxes),
                     'depth_status': d_status, 'lidar_status': l_status,
                     'flags': ';'.join(flags)})
    with open(os.path.join(out, 'frames.csv'), 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print('%d frames -> %s (see frames.csv for flags)' % (len(rows), out))


if __name__ == '__main__':
    main()
