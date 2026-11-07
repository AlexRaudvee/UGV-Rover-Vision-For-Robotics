#!/usr/bin/env python3
"""Time the depth detector's steps per frame: colour-only (run 6) vs ground check.

Runs the detection logic only (no ROS), single-threaded like the node, on recorded
frames. To benchmark on the robot without copying the whole bag, export frames first:

    # on the laptop (ROS sourced): save every 4th frame up to 39 s
    python3 scripts/benchmark_detection.py --save frames.npz ~/bags/avoidance_cleaned_v2.mcap
    # on the robot / anywhere with object_detection installed
    python3 scripts/benchmark_detection.py frames.npz

Prints median / 95th percentile / max milliseconds per step.
"""

import argparse
import collections
import os
import time

import cv2
import numpy as np
from object_detection.floor import FloorSettings, FloorTracker, prior_plane
import object_detection.grounded as grounded
from object_detection.node import pitch_correction
from object_detection.perception import (CameraModel, depth_estimate_for_blob,
                                         DetectorSettings, find_black_blobs, measure_blob,
                                         merge_detections)
import yaml

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src',
                      'object_detection', 'config', 'detector.yaml')


def export(bag, out, every=4, max_t=39.0, pitch=5.0):
    """Save resized RGB, depth, intrinsics and the floor prior from a bag to .npz."""
    from export_detection_overlays import open_reader
    from export_sensor_overlays import _Bridge, transform_arrays
    from object_detection.image_io import decode_depth
    from rclpy.serialization import deserialize_message
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, CompressedImage, Image
    import tf2_ros
    from tf2_msgs.msg import TFMessage
    buffer, info, rgbs, depths = tf2_ros.Buffer(), None, [], {}
    reader = open_reader(bag, ['/tf_static', '/oak/stereo/image_raw', '/oak/rgb/camera_info',
                               '/oak/rgb/image_raw/compressed'])
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic == '/tf_static':
            for transform in deserialize_message(raw, TFMessage).transforms:
                buffer.set_transform_static(transform, 'bag')
        elif topic == '/oak/rgb/camera_info':
            info = info or deserialize_message(raw, CameraInfo)
        elif topic == '/oak/rgb/image_raw/compressed':
            msg = deserialize_message(raw, CompressedImage)
            rgbs.append((msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec, msg))
        else:
            msg = deserialize_message(raw, Image)
            depths[msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec] = msg
    keys = np.array(sorted(depths))
    t0 = rgbs[0][0]
    rotation, translation = transform_arrays(buffer.lookup_transform(
        'oak_rgb_camera_optical_frame', 'base_footprint', Time()).transform)
    fix = pitch_correction(pitch)
    bgrs, deps = [], []
    for key, msg in rgbs[::every]:
        if (key - t0) / 1e9 > max_t:
            break
        bgr = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        bgr = cv2.resize(bgr, (640, 360), interpolation=cv2.INTER_AREA)
        depth = decode_depth(depths[keys[np.argmin(abs(keys - key))]], _Bridge())
        bgrs.append(bgr)
        deps.append(cv2.resize(depth, (640, 360), interpolation=cv2.INTER_NEAREST))
    camera = CameraModel.from_info(info, 640, 360, rectified=False)
    np.savez_compressed(out, bgr=np.array(bgrs), depth=np.array(deps, np.float32),
                        matrix=camera.matrix, distortion=camera.distortion,
                        rotation=fix @ rotation, translation=fix @ translation)
    print('%d frames -> %s' % (len(bgrs), out))


def benchmark(path):
    data = np.load(path)
    cv2.setNumThreads(1)   # like the node
    cfg = yaml.safe_load(open(CONFIG))['/**']['ros__parameters']
    fields = DetectorSettings.__dataclass_fields__
    new = DetectorSettings(**{k: v for k, v in cfg.items() if k in fields})
    old = DetectorSettings(**{k: v for k, v in cfg.items() if k in fields})
    old.ground_check = False
    camera = CameraModel(640, 360, data['matrix'], data['distortion'])
    prior = prior_plane(data['rotation'], data['translation'])
    tracker = FloorTracker(FloorSettings(sample_step=4 // grounded.SCALE,
                                         min_inliers=new.floor_min_inliers))
    plane = None
    times = collections.defaultdict(list)

    frame_total = collections.Counter()

    def timed(label, fn, *args, **kwargs):
        start = time.perf_counter()
        result = fn(*args, **kwargs)
        elapsed = (time.perf_counter() - start) * 1000
        times[label].append(elapsed)
        frame_total[label.split(':')[0]] += elapsed
        return result

    for index, (bgr, depth) in enumerate(zip(data['bgr'], data['depth'])):
        blobs = timed('old: colour segmentation', find_black_blobs, bgr, camera, old)[0]
        estimates = timed('old: distance per region', lambda: [
            depth_estimate_for_blob(blob, depth, old) for blob in blobs])
        timed('old: measure + merge', lambda: merge_detections([item for item in (
            measure_blob(b, e.axial_m, 'depth', old, e) for b, e in zip(blobs, estimates))
            if item], old))
        blobs = timed('new: colour segmentation', find_black_blobs, bgr, camera, new,
                      keep_small=True)[0]
        points = timed('new: depth -> 3D points', grounded.working_points, depth,
                       camera.matrix)
        if plane is None or index % new.floor_refit_frames == 0:   # like the node
            plane = timed('new: floor fit (when run)', tracker.update, points, prior)
        heights = timed('new: heights', plane.height, points) if plane else None
        objects = timed('new: ground check', grounded.ground_objects, blobs, bgr, depth,
                        heights, camera, new)[0]
        timed('new: measure + merge', lambda: merge_detections([item for item in (
            measure_blob(o.blob, o.estimate.axial_m, 'depth', new, o.estimate)
            for o in objects) if item], new))
        for prefix in ('old', 'new'):
            times[prefix + ': TOTAL'].append(frame_total.pop(prefix, 0.0))
    print('%d frames, 640x360, 1 thread' % len(data['bgr']))
    print('%-32s %8s %8s %8s  (ms)' % ('step', 'median', 'p95', 'max'))
    for label, values in times.items():
        values = np.array(values)
        print('%-32s %8.2f %8.2f %8.2f' % (label, np.median(values),
                                          np.percentile(values, 95), values.max()))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('input', help='frames .npz, or a bag with --save')
    parser.add_argument('--save', help='export frames from the bag to this .npz')
    args = parser.parse_args()
    if args.save:
        export(os.path.expanduser(args.input), args.save)
    else:
        benchmark(os.path.expanduser(args.input))


if __name__ == '__main__':
    main()
