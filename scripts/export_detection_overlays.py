#!/usr/bin/env python3
"""Draw every blob the detectors considered on the clean camera image, one JPEG per frame.

Left panel = depth detector, right panel = lidar detector. In both panels:
  green  = accepted detection (distance measured, width within the band)
  red    = wrong size (distance measured, width outside the band)
  yellow = no distance, or clipped by the image border

The detections JSON only stores boxes for measured objects, so unmeasured blobs are
recovered by re-running the detectors' own segmentation on the same image bytes.
Pass --grey-off for runs recorded before the grey colour rule existed.

Usage (ROS and the workspace sourced):
    python3 scripts/export_detection_overlays.py ~/bags/eval_run3 \
        ~/bags/avoidance_cleaned_v2.mcap [out_dir] [--grey-off]
"""

import json
import os
import sys

import cv2
import numpy as np
from object_detection.perception import CameraModel, DetectorSettings, find_black_blobs
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import String

RGB_TOPICS = {'/oak/rgb/image_rect/compressed': True,    # topic: rectified
              '/oak/rgb/image_raw/compressed': False}
INFO_TOPIC = '/oak/rgb/camera_info'
MAX_WIDTH = 640   # the detectors' max_width: images are shrunk to this before segmentation
SOURCES = {'/object_detection/detections': 'depth', '/obstacle_detection/detections': 'lidar'}
GREEN, YELLOW, RED = (0, 220, 0), (0, 230, 255), (0, 0, 255)


def open_reader(path, topics):
    path = os.path.expanduser(path)
    is_mcap = path.endswith('.mcap') or (
        os.path.isdir(path) and any(f.endswith('.mcap') for f in os.listdir(path)))
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap' if is_mcap else 'sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return reader


def stamp_key(stamp):
    return stamp['sec'] * 1_000_000_000 + stamp['nanosec']


def label(img, x, y, text, colour):
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
    ty = max(y - 5, 40)
    cv2.rectangle(img, (x, ty - th - 4), (x + tw + 4, ty + 3), (0, 0, 0), -1)
    cv2.putText(img, text, (x + 2, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.42, colour, 1, cv2.LINE_AA)


def classify(blobs, det):
    """Return (bbox, colour, text) for every blob, using the detector's own verdicts."""
    measured = {tuple(item['bbox']): item for item in det['blockers']}
    items = []
    for blob in blobs:
        item = measured.get(tuple(blob.bbox))
        if item and item['accepted']:
            items.append((blob.bbox, GREEN,
                          '%.2fm %.0fcm' % (item['distance_m'], item['width_m'] * 100)))
        elif item:
            items.append((blob.bbox, RED, 'wrong size %.2fm %.0fcm'
                          % (item['distance_m'], item['width_m'] * 100)))
        else:
            items.append((blob.bbox, YELLOW, 'clipped' if blob.clipped else 'no distance'))
    return items


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    eval_bag, input_bag = args[0], args[1]
    out = os.path.expanduser(args[2] if len(args) > 2 else eval_bag.rstrip('/') + '_detections')
    os.makedirs(out, exist_ok=True)
    # Same segmentation settings as the replay (bag_replay.yaml width band).
    settings = DetectorSettings(width_min_m=0.23, width_max_m=0.33)
    if '--grey-off' in sys.argv:
        settings.grey_value_max = 0

    frames = {}
    reader = open_reader(eval_bag, list(SOURCES))
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        det = json.loads(deserialize_message(raw, String).data)
        if det.get('fresh') and det.get('stamp'):
            frames.setdefault(stamp_key(det['stamp']), {})[topic] = det

    # Use the colour topic the detectors read: rectified if the bag has it, else raw.
    probe = open_reader(input_bag, list(RGB_TOPICS))
    rgb_topic = probe.read_next()[0] if probe.has_next() else None
    if rgb_topic is None:
        sys.exit('No OAK colour image topic in ' + input_bag)
    del probe
    info, t0, written, mismatched = None, None, 0, 0
    reader = open_reader(input_bag, [rgb_topic, INFO_TOPIC])
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic == INFO_TOPIC:
            info = info or deserialize_message(raw, CameraInfo)
            continue
        msg = deserialize_message(raw, CompressedImage)
        key = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        t0 = key if t0 is None else t0
        if key not in frames or info is None:
            continue   # detectors run at 5 Hz, so most camera frames were not processed
        clean = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        factor = min(1.0, MAX_WIDTH / clean.shape[1])   # same resize as the detector node
        if factor < 1.0:
            clean = cv2.resize(clean, None, fx=factor, fy=factor, interpolation=cv2.INTER_AREA)
        camera = CameraModel.from_info(info, clean.shape[1], clean.shape[0],
                                       rectified=RGB_TOPICS[rgb_topic])
        blobs, _ = find_black_blobs(clean, camera, settings)
        t = (key - t0) / 1e9
        panels = []
        for topic, name in SOURCES.items():
            img = clean.copy()
            det = frames[key].get(topic)
            if det is None:
                title = '%s: no message' % name.upper()
            else:
                if det.get('blob_count') is not None and det['blob_count'] != len(blobs):
                    mismatched += 1
                items = classify(blobs, det)
                for (x, y, w, h), colour, _ in items:   # boxes first, labels on top
                    cv2.rectangle(img, (x, y), (x + w, y + h), colour, 2)
                for (x, y, w, h), colour, text in items:
                    label(img, x, y, text, colour)
                accepted = sum(colour == GREEN for _, colour, _ in items)
                title = '%s: %d accepted, %d not accepted' % (
                    name.upper(), accepted, len(items) - accepted)
            cv2.rectangle(img, (0, 0), (img.shape[1], 26), (0, 0, 0), -1)
            cv2.putText(img, title, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1,
                        cv2.LINE_AA)
            cv2.putText(img, 't=%.1fs' % t, (8, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 255, 255), 1, cv2.LINE_AA)
            panels.append(img)
        divider = np.full((clean.shape[0], 4, 3), 255, np.uint8)
        cv2.imwrite(os.path.join(out, 't%05.1f.jpg' % t),
                    np.hstack([panels[0], divider, panels[1]]), [cv2.IMWRITE_JPEG_QUALITY, 90])
        written += 1
    print('%d frames -> %s' % (written, out))
    if mismatched:
        print('WARNING: %d panels where re-segmentation found a different number of blobs than '
              'the detector (wrong --grey-off setting?)' % mismatched)


if __name__ == '__main__':
    main()
