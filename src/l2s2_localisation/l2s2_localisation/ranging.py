"""AprilTag distance and robust observation filtering, independent of ROS."""

from collections import deque
import math

import numpy as np

from l2s1_marker_detection.native_apriltag import estimate_pose


def wrap(angle):
    return (angle + math.pi) % math.tau - math.pi


def base_to_map(point, robot):
    c, s = math.cos(robot[2]), math.sin(robot[2])
    return np.array([robot[0]+c*point[0]-s*point[1],
                     robot[1]+s*point[0]+c*point[1], point[2]])


def planar_pose(transform, pose):
    position = base_to_map([pose[0], pose[1], 0], transform)
    return (float(position[0]), float(position[1]), wrap(transform[2]+pose[2]))


class PoseHistory:
    def __init__(self, max_skew=.25):
        self.samples = deque(maxlen=300)
        self.max_skew = max_skew

    def add(self, stamp, pose):
        if self.samples and stamp < self.samples[-1][0]:
            self.samples.clear()
        if self.samples and stamp == self.samples[-1][0]:
            self.samples[-1] = (stamp, pose)
        else:
            self.samples.append((stamp, pose))

    def at(self, stamp):
        if not self.samples:
            return None
        if stamp <= self.samples[0][0]:
            return self.samples[0][1] if self.samples[0][0]-stamp <= self.max_skew else None
        if stamp >= self.samples[-1][0]:
            return self.samples[-1][1] if stamp-self.samples[-1][0] <= self.max_skew else None
        for (ta, a), (tb, b) in zip(self.samples, list(self.samples)[1:]):
            if ta <= stamp <= tb:
                if min(stamp-ta, tb-stamp) > self.max_skew:
                    return None
                ratio = (stamp-ta)/(tb-ta)
                return (a[0]+ratio*(b[0]-a[0]), a[1]+ratio*(b[1]-a[1]),
                        wrap(a[2]+ratio*wrap(b[2]-a[2])))
        return None


def observe_tag(item, size, camera, image_size, mount, pan, tilt, config):
    if item.get('hamming', 0) > config['max_hamming']:
        raise ValueError('corrected-bit count exceeds the configured limit')
    if item.get('decision_margin', float('inf')) < config['min_decision_margin']:
        raise ValueError('AprilTag decision margin is too low')
    corners = np.asarray(item['corners'], float).reshape(4, 2)
    side = float(np.linalg.norm(corners-np.roll(corners, -1, axis=0), axis=1).mean())
    if side < config['min_side_px']:
        raise ValueError('tag is too small for a distance measurement')
    pose = estimate_pose(corners, size, camera.matrix_for(*image_size), camera.D)
    if pose['reprojection_error_px'] > config['max_reprojection_error_px']:
        raise ValueError('pose reprojection error exceeds the configured limit')
    translation = pose['translation']
    position = mount.camera_center(pan, tilt) + mount.camera_rotation(pan, tilt) @ translation
    if not np.all(np.isfinite(position)) or np.linalg.norm(translation) > config['max_range_m']:
        raise ValueError('pose distance is invalid or exceeds maximum range')
    return {'position': position, 'camera_distance_m': float(np.linalg.norm(translation)),
            'side_px': side, 'reprojection_error_px': pose['reprojection_error_px'],
            'object_error': pose['object_error']}


class SpatialFilter:
    """Median fixed-frame points, so robot motion does not lag a filtered range."""
    def __init__(self, count=9, window=1.0):
        self.samples = deque(maxlen=count)
        self.window = window
        self.frame = None

    def add(self, stamp, point, frame):
        if frame != self.frame or (self.samples and stamp < self.samples[-1][0]):
            self.samples.clear()
        self.frame = frame
        while self.samples and stamp-self.samples[0][0] > self.window:
            self.samples.popleft()
        point = np.asarray(point, float)
        if len(self.samples) >= 3:
            previous = np.array([p for _, p in self.samples])
            median = np.median(previous, axis=0)
            deviations = np.linalg.norm(previous-median, axis=1)
            # Broad physical gate + median filtering; never lock onto a single frame.
            limit = max(.5, 6 * float(np.median(deviations)))
            if np.linalg.norm(point-median) > limit:
                raise ValueError('distance outlier rejected by temporal filter')
        self.samples.append((stamp, point))
        points = np.array([p for _, p in self.samples])
        return np.median(points, axis=0), points
