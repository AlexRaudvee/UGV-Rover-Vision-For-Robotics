#!/usr/bin/env python3
"""Add the raw sensor data under each detection overlay frame.

Top row:    the existing frame from export_detection_overlays.py (depth | lidar panels).
Bottom row: left  = the stereo depth image (colour = distance),
            right = the lidar scan projected into the camera image (colour = distance).
The detection boxes from the top row are drawn in white on both, for reference.

The depth image and scan used are the ones nearest in time to the camera frame
(within 0.12 s, the detectors' sync tolerance), with the same camera-pitch fix as the
replay. Colours: red = near (0.15 m) ... blue = far (5 m); black = no depth.

Usage (ROS sourced):
    python3 scripts/export_sensor_overlays.py ~/bags/eval_run6_merge \
        ~/bags/avoidance_cleaned_v2.mcap [frames_dir] [out_dir] [--pitch-down 5.0]
"""

import bisect
import json
import os
import sys

import cv2
import numpy as np
from object_detection.floor import (classify, depth_points, FloorSettings, FloorTracker,
                                    prior_plane)
from object_detection.image_io import decode_depth
from object_detection.node import pitch_correction, transform_arrays
from object_detection.perception import CameraModel, DetectorSettings, scan_points_in_camera
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, LaserScan
from std_msgs.msg import String
import tf2_ros
from tf2_msgs.msg import TFMessage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from export_detection_overlays import INFO_TOPIC, MAX_WIDTH, open_reader  # noqa: E402

RGB_TOPIC = '/oak/rgb/image_raw/compressed'
DEPTH_TOPIC, SCAN_TOPIC = '/oak/stereo/image_raw', '/scan'
SOURCES = {'/object_detection/detections': 0, '/obstacle_detection/detections': 1}
NEAR, FAR, SYNC_S = 0.15, 5.0, 0.12


MAX_T = 39.0   # s; ignore the rest of the bag (robot falls over)

def stamp(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def colour(distances):
    """Map metres to JET colours, red = near, blue = far (same as the depth preview)."""
    levels = (255 * np.clip((FAR - np.asarray(distances)) / (FAR - NEAR), 0, 1)).astype(np.uint8)
    return cv2.applyColorMap(levels.reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)


def nearest(keys, values, key):
    i = bisect.bisect_left(keys, key)
    best = min((j for j in (i - 1, i) if 0 <= j < len(keys)),
               key=lambda j: abs(keys[j] - key), default=None)
    if best is None or abs(keys[best] - key) > SYNC_S * 1e9:
        return None, None
    return values[best], (keys[best] - key) / 1e9


def title(text, width, height=24):
    strip = np.zeros((height, width, 3), np.uint8)
    cv2.putText(strip, text, (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
                cv2.LINE_AA)
    return strip


def legend(width):
    """Colour bar: near (left) to far (right)."""
    strip = np.zeros((26, width, 3), np.uint8)
    bar = colour(np.linspace(NEAR, FAR, width - 120)).reshape(1, -1, 3)
    strip[6:20, 60:width - 60] = np.repeat(bar, 14, axis=0)
    for text, x in (('%.2gm' % NEAR, 4), ('%gm' % FAR, width - 54)):
        cv2.putText(strip, text, (x, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1,
                    cv2.LINE_AA)
    for metres in (1, 2, 3, 4):
        x = 60 + int((metres - NEAR) / (FAR - NEAR) * (width - 120))
        cv2.putText(strip, '%d' % metres, (x - 3, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                    (0, 0, 0), 1, cv2.LINE_AA)
    return strip


def colour_row(bgr, settings=DetectorSettings()):
    """Brightness (V) and saturation (S) channels used by the colour mask.

    A pixel is a box candidate when V <= value_max AND S <= saturation_max; passing
    pixels are tinted magenta. The cyan outline is the final mask (both tests, then the
    3x3 open/close cleanup) that the blobs are cut from.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 0, 0), (179, settings.saturation_max, settings.value_max))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    width = bgr.shape[1]
    columns = []
    for channel, limit, name in ((2, settings.value_max, 'V (brightness)'),
                                 (1, settings.saturation_max, 'S (saturation)')):
        values = hsv[:, :, channel]
        passing = values <= limit
        image = cv2.cvtColor(values, cv2.COLOR_GRAY2BGR)
        image[passing] = (0.5 * image[passing] + 0.5 * np.array([255, 0, 255])).astype(np.uint8)
        cv2.drawContours(image, contours, -1, (255, 255, 0), 1)
        text = '%s: magenta = %s <= %g (%.0f%% of pixels)   cyan = final mask' % (
            name, name[0], limit, 100 * passing.mean())
        columns.append(np.vstack([title(text, width), image]))
    return np.hstack([columns[0], np.full((columns[0].shape[0], 4, 3), 255, np.uint8),
                      columns[1]])


HEIGHT_MAX = 0.5   # m; colour scale for objects above the floor


def floor_row(bgr, depth, camera, prior, tracker, settings=FloorSettings()):
    """Floor (green, incl. below the plane) vs above floor (yellow->red by height)."""
    width, height = bgr.shape[1], bgr.shape[0]
    overlay = (bgr * 0.5).astype(np.uint8)
    classes = np.zeros_like(bgr)
    if depth is None:
        text = 'FLOOR: no depth image'
        heights = np.full(bgr.shape[:2], np.nan)
    else:
        points = depth_points(depth, camera.matrix)
        plane = tracker.update(points, prior)
        if plane is None:
            heights = np.full(bgr.shape[:2], np.nan)
            text = 'FLOOR: NOT FOUND (no large flat area near the expected floor)'
        else:
            heights = plane.height(points)
            text = ('FLOOR: %s%s   camera %.3f m above floor, pitch down %.1f, roll %.1f deg'
                    % (plane.source, ' (%d pts)' % plane.inliers if plane.source == 'fit'
                       else '', plane.camera_height_m, plane.pitch_down_deg, plane.roll_deg))
    floor, above = classify(heights, settings)
    level = np.clip((heights - settings.floor_tolerance_m)
                    / (HEIGHT_MAX - settings.floor_tolerance_m), 0, 1)
    # Yellow (just above floor) -> red (HEIGHT_MAX and higher), BGR.
    above_bgr = np.dstack((np.zeros_like(level), 255 * (1 - level), np.full_like(level, 255)))
    for image, alpha in ((overlay, 0.55), (classes, 1.0)):
        image[floor] = ((1 - alpha) * image[floor]
                        + alpha * np.array([0, 200, 0])).astype(np.uint8)
        image[above] = ((1 - alpha) * image[above] + alpha * above_bgr[above]).astype(np.uint8)
    share = '%.0f%% floor, %.0f%% above floor, %.0f%% no/bad depth' % (
        100 * floor.mean(), 100 * above.mean(), 100 * (1 - floor.mean() - above.mean()))
    key = np.zeros((26, width, 3), np.uint8)
    bar = np.dstack((np.zeros(width - 300), 255 * (1 - np.linspace(0, 1, width - 300)),
                     np.full(width - 300, 255))).astype(np.uint8)
    key[6:20, 150:width - 150] = np.repeat(bar, 14, axis=0)
    for label_text, x, c in (('floor <=%gcm' % (100 * settings.floor_tolerance_m), 4,
                              (0, 200, 0)),
                             ('%gcm' % (100 * settings.floor_tolerance_m), 112, (255,) * 3),
                             ('%gcm+' % (100 * HEIGHT_MAX), width - 145, (255,) * 3)):
        cv2.putText(key, label_text, (x, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.4, c, 1, cv2.LINE_AA)
    columns = [np.vstack([title(text, width), overlay, key]),
               np.vstack([title(share + '   (black)', width), classes, key])]
    return np.hstack([columns[0], np.full((columns[0].shape[0], 4, 3), 255, np.uint8),
                      columns[1]])


def boxes(image, detections):
    for det in detections:
        x, y, w, h = det['bbox']
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 255), 1)


def main():
    args = sys.argv[1:]
    pitch = 5.0
    if '--pitch-down' in args:
        i = args.index('--pitch-down')
        pitch, args = float(args[i + 1]), args[:i] + args[i + 2:]
    eval_bag, input_bag = (os.path.expanduser(a) for a in args[:2])
    frames_dir = os.path.expanduser(args[2] if len(args) > 2
                                    else eval_bag.rstrip('/') + '_detections')
    out = os.path.expanduser(args[3] if len(args) > 3
                             else eval_bag.rstrip('/') + '_sensors')
    os.makedirs(out, exist_ok=True)

    detections = {}   # rgb stamp -> [depth detections, lidar detections]
    reader = open_reader(eval_bag, list(SOURCES))
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        det = json.loads(deserialize_message(raw, String).data)
        if det.get('fresh') and det.get('stamp'):
            key = det['stamp']['sec'] * 1_000_000_000 + det['stamp']['nanosec']
            detections.setdefault(key, [[], []])[SOURCES[topic]] = det['detections']

    buffer = tf2_ros.Buffer()
    depth_keys, depth_msgs, scan_keys, scan_msgs = [], [], [], []
    rgb_keys, info = [], None
    reader = open_reader(input_bag, ['/tf_static', DEPTH_TOPIC, SCAN_TOPIC, RGB_TOPIC,
                                     INFO_TOPIC])
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic == '/tf_static':
            for transform in deserialize_message(raw, TFMessage).transforms:
                buffer.set_transform_static(transform, 'bag')
        elif topic == DEPTH_TOPIC:
            msg = deserialize_message(raw, Image)
            depth_keys.append(stamp(msg.header))
            depth_msgs.append(msg)
        elif topic == SCAN_TOPIC:
            msg = deserialize_message(raw, LaserScan)
            scan_keys.append(stamp(msg.header))
            scan_msgs.append(msg)
        elif topic == INFO_TOPIC:
            info = info or deserialize_message(raw, CameraInfo)
        else:
            msg = deserialize_message(raw, CompressedImage)
            rgb_keys.append((stamp(msg.header), msg))
    order = np.argsort(depth_keys)
    depth_keys, depth_msgs = [depth_keys[i] for i in order], [depth_msgs[i] for i in order]
    order = np.argsort(scan_keys)
    scan_keys, scan_msgs = [scan_keys[i] for i in order], [scan_msgs[i] for i in order]
    t0 = rgb_keys[0][0]   # same time origin as export_detection_overlays.py

    camera_frame = 'oak_rgb_camera_optical_frame'
    lidar_frame = scan_msgs[0].header.frame_id
    rotation, translation = transform_arrays(
        buffer.lookup_transform(camera_frame, lidar_frame, Time()).transform)
    fix = pitch_correction(pitch)
    rotation, translation = fix @ rotation, fix @ translation
    # Floor prior: the robot's base_footprint z = 0 plane, with the same pitch fix.
    base_rot, base_trans = transform_arrays(
        buffer.lookup_transform(camera_frame, 'base_footprint', Time()).transform)
    prior = prior_plane(fix @ base_rot, fix @ base_trans)
    tracker = FloorTracker()
    print('floor prior: camera %.3f m above floor, pitched down %.1f deg, roll %.1f deg' % (
        prior.camera_height_m, prior.pitch_down_deg, prior.roll_deg))

    written = 0
    for key, rgb in rgb_keys:
        t = (key - t0) / 1e9
        top_path = os.path.join(frames_dir, 't%05.1f.jpg' % t)
        if (key - t0) / 1e9 > MAX_T:
            continue   # avoidance bag: the robot falls after ~39 s
        if key not in detections or not os.path.exists(top_path):
            continue
        depth_dets, lidar_dets = detections[key]
        clean = cv2.imdecode(np.frombuffer(bytes(rgb.data), np.uint8), cv2.IMREAD_COLOR)
        original = clean.shape[:2]
        factor = min(1.0, MAX_WIDTH / clean.shape[1])
        if factor < 1.0:
            clean = cv2.resize(clean, None, fx=factor, fy=factor, interpolation=cv2.INTER_AREA)
        height, width = clean.shape[:2]
        camera = CameraModel.from_info(info, width, height, rectified=False)

        # Depth panel.
        depth_msg, depth_dt = nearest(depth_keys, depth_msgs, key)
        depth = None
        if depth_msg is None:
            depth_img = np.zeros_like(clean)
            depth_title = 'DEPTH IMAGE: none within %.2fs' % SYNC_S
        else:
            depth = decode_depth(depth_msg, _Bridge())
            if depth.shape != original:
                depth = cv2.resize(depth, (original[1], original[0]),
                                   interpolation=cv2.INTER_NEAREST)
            depth = cv2.resize(depth, (width, height), interpolation=cv2.INTER_NEAREST)
            valid = np.isfinite(depth)
            depth_img = np.zeros_like(clean)
            depth_img[valid] = colour(depth[valid])
            depth_title = 'DEPTH IMAGE (dt=%+.2fs)   %.0f%% pixels valid' % (
                depth_dt, 100 * valid.mean())
        boxes(depth_img, depth_dets)

        # Lidar panel: dimmed camera image with scan points projected on it.
        scan, scan_dt = nearest(scan_keys, scan_msgs, key)
        lidar_img = (clean * 0.45).astype(np.uint8)
        if scan is None:
            lidar_title = 'LIDAR SCAN: none within %.2fs' % SYNC_S
        else:
            points = scan_points_in_camera(scan, rotation, translation)
            points = points[points[:, 2] > 0.05]
            on_image = 0
            if len(points):
                pixels = camera.project(points)
                inside = ((pixels[:, 0] >= 0) & (pixels[:, 0] < width)
                          & (pixels[:, 1] >= 0) & (pixels[:, 1] < height))
                distances = np.linalg.norm(points[inside], axis=1)
                for (u, v), bgr in zip(pixels[inside], colour(distances)):
                    cv2.circle(lidar_img, (int(round(u)), int(round(v))), 3,
                               tuple(int(c) for c in bgr), -1, cv2.LINE_AA)
                on_image = int(inside.sum())
            lidar_title = 'LIDAR SCAN (dt=%+.2fs)   %d points in view' % (scan_dt, on_image)
        boxes(lidar_img, lidar_dets)

        bottom_columns = [np.vstack([title(text, width), img, legend(width)])
                          for text, img in ((depth_title, depth_img), (lidar_title, lidar_img))]
        bottom = np.hstack([bottom_columns[0],
                            np.full((bottom_columns[0].shape[0], 4, 3), 255, np.uint8),
                            bottom_columns[1]])
        bottom = np.vstack([colour_row(clean), np.full((4, bottom.shape[1], 3), 255, np.uint8),
                            bottom, np.full((4, bottom.shape[1], 3), 255, np.uint8),
                            floor_row(clean, depth, camera, prior, tracker)])
        top = cv2.imread(top_path)
        if top.shape[1] != bottom.shape[1]:
            bottom = cv2.resize(bottom, (top.shape[1], int(bottom.shape[0] * top.shape[1]
                                                          / bottom.shape[1])))
        separator = np.full((4, top.shape[1], 3), 255, np.uint8)
        cv2.imwrite(os.path.join(out, 't%05.1f.jpg' % t), np.vstack([top, separator, bottom]),
                    [cv2.IMWRITE_JPEG_QUALITY, 90])
        written += 1
    print('%d frames -> %s' % (written, out))


class _Bridge:
    """Minimal stand-in for cv_bridge for raw 16UC1 / 32FC1 depth images."""

    def imgmsg_to_cv2(self, message, desired_encoding='passthrough'):
        dtype = np.uint16 if message.encoding == '16UC1' else np.float32
        return np.frombuffer(bytes(message.data), dtype).reshape(
            message.height, message.step // np.dtype(dtype).itemsize)[:, :message.width]


if __name__ == '__main__':
    main()
