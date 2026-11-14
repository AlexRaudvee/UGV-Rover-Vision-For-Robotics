"""Marker pose from corners and pan-tilt camera geometry (no ROS imports).

Frames
  optical: OpenCV camera frame, x right, y down, z forward (solvePnP output).
  robot:   base_footprint, x forward, y left, z up (REP 103). Angles are
           counter-clockwise positive, so a positive bearing is to the LEFT.
  pan:     camera yaw relative to the robot, left positive.
  tilt:    camera pitch, up positive.

The mount model follows the URDF (ugv_rover.urdf): the pan axis sits about
8 mm behind base_footprint, the tilt axis about 0.211 m above the floor, and
the camera's optical centre about 48 mm in front of the tilt axis. Check these
with a tape measure; they shift the marker position by a few centimetres.
"""

from dataclasses import dataclass
import math

import cv2
import numpy as np

# Columns: optical x (right) -> -left, optical y (down) -> -up, optical z -> forward.
ROBOT_FROM_OPTICAL = np.array([[0.0, 0.0, 1.0],
                               [-1.0, 0.0, 0.0],
                               [0.0, -1.0, 0.0]])


def rot_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_y(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


@dataclass(frozen=True)
class MountModel:
    """Where the pan-tilt camera sits on the robot (metres)."""

    pan_pivot_xy: tuple = (-0.008, 0.0)
    tilt_axis_height: float = 0.211
    camera_offset: float = 0.048

    def head_rotation(self, pan, tilt):
        """Camera body (x forward, y left, z up) -> robot frame."""
        return rot_z(pan) @ rot_y(-tilt)

    def camera_rotation(self, pan, tilt):
        """Optical frame -> robot frame."""
        return self.head_rotation(pan, tilt) @ ROBOT_FROM_OPTICAL

    def camera_center(self, pan, tilt):
        pivot = np.array([self.pan_pivot_xy[0], self.pan_pivot_xy[1], self.tilt_axis_height])
        return pivot + self.head_rotation(pan, tilt) @ np.array([self.camera_offset, 0.0, 0.0])


def square_object_points(size):
    """Marker corners in the marker frame, in OpenCV's corner order (for IPPE_SQUARE)."""
    half = size / 2.0
    return np.array([[-half, half, 0.0], [half, half, 0.0],
                     [half, -half, 0.0], [-half, -half, 0.0]])


def marker_translation(corners_px, size, camera, image_size):
    """Marker centre in the optical frame from its 4 raw-image corners (solvePnP)."""
    matrix = camera.matrix_for(*image_size)
    image_points = np.asarray(corners_px, np.float64).reshape(4, 1, 2)
    ok, _, tvec = cv2.solvePnP(square_object_points(size), image_points, matrix, camera.D,
                               flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        raise ValueError('solvePnP failed')
    return tvec.ravel()


@dataclass
class MarkerObservation:
    """One sighting of a marker, expressed relative to the robot."""

    family: str
    marker_id: int
    pan: float                  # rad, left positive
    tilt: float                 # rad, up positive
    camera_distance: float      # m, camera centre to marker centre (|tvec|)
    position: np.ndarray        # m, marker centre in the robot frame (x, y, z)
    centre_offset: float        # 0 at image centre, 1 at the left/right image edge
    side_px: float
    stamp: float = 0.0

    @property
    def range(self):
        """Floor distance from the robot centre to the marker (m)."""
        return float(math.hypot(self.position[0], self.position[1]))

    @property
    def bearing(self):
        """Direction to the marker from the robot's heading (rad, left positive)."""
        return float(math.atan2(self.position[1], self.position[0]))


def observe(family, marker_id, corners_px, size, camera, image_size, mount, pan, tilt,
            stamp=0.0):
    """Turn detected corners plus the commanded pan/tilt into a MarkerObservation."""
    corners = np.asarray(corners_px, float).reshape(4, 2)
    tvec = marker_translation(corners, size, camera, image_size)
    position = mount.camera_center(pan, tilt) + mount.camera_rotation(pan, tilt) @ tvec
    half_width = image_size[0] / 2.0
    side = float(np.linalg.norm(corners - np.roll(corners, -1, axis=0), axis=1).mean())
    return MarkerObservation(family, int(marker_id), pan, tilt, float(np.linalg.norm(tvec)),
                             position, abs(corners[:, 0].mean() - half_width) / half_width,
                             side, stamp)


def summarize(observations):
    """One entry per marker from all sightings in a scan.

    A marker can be seen at two neighbouring pan stops. The stop where it is
    nearest the image centre is used (least lens distortion), and the median
    over that stop's frames is reported, with the spread as a noise estimate.
    """
    groups = {}
    for item in observations:
        groups.setdefault((item.family, item.marker_id), []).append(item)
    summary = []
    for (family, marker_id), items in sorted(groups.items()):
        by_pan = {}
        for item in items:
            by_pan.setdefault(round(item.pan, 4), []).append(item)
        best = min(by_pan.values(), key=lambda group: np.mean([i.centre_offset for i in group]))
        position = np.median([i.position for i in best], axis=0)
        ranges = [i.range for i in best]
        summary.append({
            'family': family, 'id': marker_id,
            'range_m': round(float(math.hypot(position[0], position[1])), 3),
            'bearing_deg': round(math.degrees(math.atan2(position[1], position[0])), 2),
            'x_m': round(float(position[0]), 3), 'y_m': round(float(position[1]), 3),
            'height_m': round(float(position[2]), 3),
            'camera_distance_m': round(float(np.median([i.camera_distance for i in best])), 3),
            'range_std_m': round(float(np.std(ranges)), 4),
            'frames': len(best), 'pan_deg': round(math.degrees(best[0].pan), 1),
            'tilt_deg': round(math.degrees(best[0].tilt), 1),
            'side_px': round(float(np.median([i.side_px for i in best])), 1),
            'seen_at_pans_deg': sorted(round(math.degrees(p), 1) for p in by_pan),
        })
    return summary
