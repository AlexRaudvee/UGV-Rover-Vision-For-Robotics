#!/usr/bin/env python3
"""Draw only the final accepted detections on the clean camera image, one JPEG per frame.

Left panel = depth detector (green), right panel = lidar detector (yellow).

Usage (ROS sourced):
    python3 scripts/export_detection_overlays.py ~/bags/eval_run3 \
        ~/bags/avoidance_cleaned_v2.mcap [out_dir]
"""

import json
import os
import sys

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

RGB_TOPIC = '/oak/rgb/image_raw/compressed'
SOURCES = {  # topic: (label, BGR colour)
    '/object_detection/detections': ('depth', (0, 220, 0)),
    '/obstacle_detection/detections': ('lidar', (0, 230, 255)),
}


def open_reader(path):
    path = os.path.expanduser(path)
    is_mcap = path.endswith('.mcap') or (
        os.path.isdir(path) and any(f.endswith('.mcap') for f in os.listdir(path)))
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap' if is_mcap else 'sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    return reader


def stamp_key(stamp):
    return stamp['sec'] * 1_000_000_000 + stamp['nanosec']


def main():
    eval_bag, input_bag = sys.argv[1], sys.argv[2]
    out = os.path.expanduser(sys.argv[3] if len(sys.argv) > 3
                             else eval_bag.rstrip('/') + '_detections')
    os.makedirs(out, exist_ok=True)

    # Accepted detections per camera frame, keyed by the RGB acquisition stamp.
    frames = {}
    reader = open_reader(eval_bag)
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(SOURCES)))
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        det = json.loads(deserialize_message(raw, String).data)
        if det.get('fresh') and det.get('stamp'):
            frames.setdefault(stamp_key(det['stamp']), {})[topic] = det['detections']

    reader = open_reader(input_bag)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[RGB_TOPIC]))
    t0, written = None, 0
    while reader.has_next():
        _, raw, _ = reader.read_next()
        msg = deserialize_message(raw, CompressedImage)
        key = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        t0 = key if t0 is None else t0
        if key not in frames:
            continue   # detectors run at 5 Hz, so most camera frames were not processed
        clean = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        scale = clean.shape[1] / 640.0   # detector bboxes are in 640-px-wide coordinates
        t = (key - t0) / 1e9
        panels = []
        for topic, (label, colour) in SOURCES.items():   # one panel per detector
            img = clean.copy()
            boxes = frames[key].get(topic)
            for d in boxes or []:
                x, y, w, h = (int(round(v * scale)) for v in d['bbox'])
                cv2.rectangle(img, (x, y), (x + w, y + h), colour, 2)
                text = '%.2fm %.0fcm' % (d['distance_m'], d['width_m'] * 100)
                (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
                ty = max(y - 6, 40)
                cv2.rectangle(img, (x, ty - th - 4), (x + tw + 4, ty + 3), (0, 0, 0), -1)
                cv2.putText(img, text, (x + 2, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1,
                            cv2.LINE_AA)
            title = '%s: %s' % (label.upper(), 'no message' if boxes is None
                                else '%d detection(s)' % len(boxes))
            cv2.rectangle(img, (0, 0), (img.shape[1], 26), (0, 0, 0), -1)
            cv2.putText(img, title, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 1,
                        cv2.LINE_AA)
            cv2.putText(img, 't=%.1fs' % t, (8, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 255, 255), 1, cv2.LINE_AA)
            panels.append(img)
        divider = np.full((clean.shape[0], 4, 3), 255, np.uint8)
        cv2.imwrite(os.path.join(out, 't%05.1f.jpg' % t),
                    np.hstack([panels[0], divider, panels[1]]), [cv2.IMWRITE_JPEG_QUALITY, 90])
        written += 1
    print('%d frames -> %s' % (written, out))


if __name__ == '__main__':
    main()
