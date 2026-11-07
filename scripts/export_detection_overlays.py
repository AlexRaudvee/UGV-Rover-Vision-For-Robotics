#!/usr/bin/env python3
"""Draw every dark region the detectors considered on the clean camera image, per frame.

Left panel = depth detector, right panel = lidar detector. In both panels:
  green  = detection: an obstacle with a measured distance, any width. The label gives
           distance and width; 'clipped ... >=' marks a blob cut off by the left/right
           image border, whose width is only a minimum; '(N merged)' marks a detection
           merged from N pieces of one object, drawn around their combined box.
  yellow = disregarded for lack of a distance (small label 'no distance' /
           'clipped, no distance'); reported only as an unknown direction
  red    = disregarded for any other reason: 'too small' / 'hollow' / 'too thin' - a dark
           region dropped by the size filters before any distance was measured (regions
           under MIN_DRAW_AREA px, or at least 80% inside a green box, are not drawn)

The detections JSON only stores boxes for measured objects, so the other regions are
recovered by re-running the detectors' own segmentation on the same image bytes. The
re-segmentation uses the currently installed detector code, so only export runs that
were recorded with the same segmentation code.

Usage (ROS and the workspace sourced):
    python3 scripts/export_detection_overlays.py ~/bags/eval_run5 \
        ~/bags/avoidance_cleaned_v2.mcap [out_dir]
"""

import json
import math
import os
import sys

import cv2
import numpy as np
from object_detection.grounded import inside
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
MIN_DRAW_AREA = 20   # px; smaller filtered-out specks are noise and would only clutter


MAX_T = 39.0   # s; ignore the rest of the bag (robot falls over)

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


FONT = cv2.FONT_HERSHEY_SIMPLEX
LINE_H = 16   # px per text line in the strips above/below the image


def tag(img, x, y, text, colour):
    """Tiny index number at a box's top-left corner (the details go in the list below)."""
    (tw, th), _ = cv2.getTextSize(text, FONT, 0.33, 1)
    x = max(0, min(x, img.shape[1] - tw - 3))
    ty = y - 2 if y - th - 3 >= 0 else y + th + 2   # above the box, or inside if at the top
    cv2.rectangle(img, (x, ty - th - 2), (x + tw + 2, ty + 1), (0, 0, 0), -1)
    cv2.putText(img, text, (x + 1, ty), FONT, 0.33, colour, 1, cv2.LINE_AA)


COLUMNS = 3   # list entries per row in the strip below the image


def strip(lines, width, height, columns=COLUMNS):
    """Black text strip: lines = [(text, colour), ...], filled column by column."""
    out = np.zeros((height, width, 3), np.uint8)
    rows = math.ceil(len(lines) / columns)
    for i, (text, colour) in enumerate(lines):
        column, row = divmod(i, rows) if rows else (0, 0)
        cv2.putText(out, text, (6 + column * width // columns, 13 + row * LINE_H), FONT, 0.38,
                    colour, 1, cv2.LINE_AA)
    return out


def measured_items(det):
    """All objects with a measured distance (older runs kept them in 'blockers')."""
    return det['blockers'] if 'blockers' in det else det['detections']


UNKNOWN_REASONS = ('no distance', 'clipped, no distance', 'contact unknown', 'no floor found',
                   'held (distance jump)')


def detection_text(item):
    merged = ' (%d merged)' % len(item['parts']) if item.get('parts') else ''
    if item.get('seed') == 'depth':
        merged += ' [depth]'
    if item.get('clipped'):   # side-clipped: width is only a minimum
        return 'clipped %.2fm >=%.0fcm%s' % (item['distance_m'], item['width_m'] * 100, merged)
    return '%.2fm %.0fcm%s' % (item['distance_m'], item['width_m'] * 100, merged)


def classify(blobs, det):
    """Return (bbox, colour, text) for every blob: green = detection, yellow = no distance."""
    if det.get('ground_check'):
        # Ground-checked run: detections are floor-standing objects with their own boxes, and
        # every rejected colour region is listed with its reason.
        return ([(tuple(item['bbox']), GREEN, detection_text(item))
                 for item in det['detections']]
                + [(tuple(item['bbox']), YELLOW if item['reason'] in UNKNOWN_REASONS else RED,
                    item['reason']) for item in det.get('rejected', [])])
    # A merged detection lists the blobs it was built from in 'parts'; map each part to
    # it and draw it once, around its combined box.
    measured = {tuple(part): item for item in measured_items(det)
                for part in (item.get('parts') or [item['bbox']])}
    items, drawn = [], set()
    for blob in blobs:
        item = measured.get(tuple(blob.bbox))
        if item is not None:
            if id(item) in drawn:
                continue
            drawn.add(id(item))
            box = tuple(item['bbox'])
            merged = ' (%d merged)' % len(item['parts']) if item.get('parts') else ''
        if item and item.get('clipped'):   # side-clipped: width is only a minimum
            items.append((box, GREEN, 'clipped %.2fm >=%.0fcm%s'
                          % (item['distance_m'], item['width_m'] * 100, merged)))
        elif item:
            items.append((box, GREEN, '%.2fm %.0fcm%s'
                          % (item['distance_m'], item['width_m'] * 100, merged)))
        else:
            items.append((blob.bbox, YELLOW,
                          'clipped, no distance' if blob.clipped else 'no distance'))
    return items


def filtered_regions(mask, blobs, camera, settings):
    """Dark regions dropped by find_black_blobs' size filters, with the reason.

    Mirrors the filter order in object_detection.perception.find_black_blobs.
    """
    kept = {tuple(blob.bbox) for blob in blobs}
    regions = []
    for contour in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        if (x, y, w, h) in kept or area < MIN_DRAW_AREA:
            continue
        if area < settings.min_area or w < settings.min_width or h < settings.min_height:
            reason = 'too small'
        elif area / (w * h) < settings.min_fill:
            reason = 'hollow'
        else:
            rays = camera.horizontal_rays([(x, y + h / 2), (x + w, y + h / 2)])
            reason = ('too thin' if abs(float(rays[1] - rays[0])) * settings.range_max
                      < settings.width_min_m - settings.width_tolerance_m else 'dropped')
        regions.append(((x, y, w, h), RED, reason))
    return regions


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    eval_bag, input_bag = args[0], args[1]
    out = os.path.expanduser(args[2] if len(args) > 2 else eval_bag.rstrip('/') + '_detections')
    os.makedirs(out, exist_ok=True)
    # Same segmentation settings as the replay (bag_replay.yaml width band).
    settings = DetectorSettings(width_min_m=0.23, width_max_m=0.33)

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
        if (key - t0) / 1e9 > MAX_T:
            continue   # avoidance bag: the robot falls after ~39 s
        if key not in frames or info is None:
            continue   # detectors run at 5 Hz, so most camera frames were not processed
        clean = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        factor = min(1.0, MAX_WIDTH / clean.shape[1])   # same resize as the detector node
        if factor < 1.0:
            clean = cv2.resize(clean, None, fx=factor, fy=factor, interpolation=cv2.INTER_AREA)
        camera = CameraModel.from_info(info, clean.shape[1], clean.shape[0],
                                       rectified=RGB_TOPICS[rgb_topic])
        blobs, mask = find_black_blobs(clean, camera, settings)
        dropped = filtered_regions(mask, blobs, camera, settings)
        t = (key - t0) / 1e9
        panels = []   # (title, image, list lines) per detector
        for topic, name in SOURCES.items():
            img = clean.copy()
            det = frames[key].get(topic)
            lines = []
            if det is None:
                title = '%s: no message   t=%.1fs' % (name.upper(), t)
            else:
                if det.get('blob_count') is not None and det['blob_count'] != len(blobs):
                    mismatched += 1
                kept = classify(blobs, det)
                # Like the node: a dropped region lying mostly inside a detection's box is
                # part of that object, not a separate rejection.
                green = [box for box, colour, _ in kept if colour == GREEN]
                items = [region for region in dropped
                         if not any(inside(region[0], box) for box in green)]
                items += kept   # detections drawn last, on top
                for (x, y, w, h), colour, _ in items:
                    cv2.rectangle(img, (x, y), (x + w, y + h), colour,
                                  2 if colour == GREEN else 1)
                for item in det['detections']:   # object shape (ground check)
                    if item.get('outline'):
                        cv2.polylines(img, [np.array(item['outline'], np.int32)], True,
                                      (255, 255, 0), 1, cv2.LINE_AA)
                # Every box gets a tiny number; its details go in the list below the image.
                # Order: green (detections), yellow (no distance), red; each left to right.
                order = {GREEN: 0, YELLOW: 1, RED: 2}
                listed = sorted(items, key=lambda item: (order[item[1]], item[0][0]))
                for number, ((x, y, w, h), colour, text) in reversed(
                        list(enumerate(listed, 1))):   # green tags drawn last, on top
                    tag(img, x, y, str(number), colour)
                for number, (_, colour, text) in enumerate(listed, 1):
                    lines.append(('%d: %s' % (number, text), colour))
                found = sum(colour == GREEN for _, colour, _ in items)
                title = '%s: %d detection(s), %d disregarded   t=%.1fs' % (
                    name.upper(), found, len(items) - found, t)
            panels.append((title, img, lines))
        width = clean.shape[1]
        rows = max(math.ceil(len(lines) / COLUMNS) for _, _, lines in panels)
        columns = []
        for title, img, lines in panels:
            columns.append(np.vstack([strip([(title, (255, 255, 255))], width, 24)[:, :],
                                      img, strip(lines, width, max(1, rows) * LINE_H + 6)]))
        divider = np.full((columns[0].shape[0], 4, 3), 255, np.uint8)
        cv2.imwrite(os.path.join(out, 't%06.2f.jpg' % t),
                    np.hstack([columns[0], divider, columns[1]]), [cv2.IMWRITE_JPEG_QUALITY, 90])
        written += 1
    print('%d frames -> %s' % (written, out))
    if mismatched:
        print('WARNING: %d panels where re-segmentation found a different number of blobs than '
              'the detector (run recorded with different segmentation code?)' % mismatched)


if __name__ == '__main__':
    main()
