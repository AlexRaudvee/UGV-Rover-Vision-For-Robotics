"""Calibrated black silhouettes and independent metric width measurements."""

from dataclasses import asdict, dataclass
import math

import cv2
import numpy as np


@dataclass
class CameraModel:
    """Pinhole calibration scaled to the processed image resolution."""

    width: int
    height: int
    matrix: np.ndarray
    distortion: np.ndarray

    @classmethod
    def from_info(cls, info, width, height, rectified=True):
        """Use P for rectified images and K/D for raw images."""
        if info.width <= 0 or info.height <= 0:
            raise ValueError('CameraInfo has no image dimensions')
        if rectified:
            matrix = np.asarray(info.p, dtype=float).reshape(3, 4)[:, :3].copy()
            distortion = np.zeros(5)
        else:
            matrix = np.asarray(info.k, dtype=float).reshape(3, 3).copy()
            if info.distortion_model not in ('plumb_bob', 'rational_polynomial', ''):
                raise ValueError('Unsupported raw camera distortion model')
            distortion = np.asarray(info.d, dtype=float)
        if getattr(info, 'binning_x', 0) > 1 or getattr(info, 'binning_y', 0) > 1:
            raise ValueError('Binned CameraInfo requires adjusted calibration')
        roi = getattr(info, 'roi', None)
        if roi is not None and (roi.x_offset or roi.y_offset):
            raise ValueError('Cropped CameraInfo requires adjusted calibration')
        matrix[0, :] *= width / info.width
        matrix[1, :] *= height / info.height
        if (not np.isfinite(matrix).all() or not np.isfinite(distortion).all()
                or matrix[0, 0] <= 0 or matrix[1, 1] <= 0):
            raise ValueError('CameraInfo has invalid focal lengths or distortion')
        return cls(width, height, matrix, distortion)

    def horizontal_rays(self, pixels):
        """Return optical X/Z at each image point."""
        points = np.asarray(pixels, dtype=np.float32).reshape(-1, 1, 2)
        return cv2.undistortPoints(points, self.matrix, self.distortion)[:, 0, 0]

    def bearings(self, pixels):
        """Return bearings in radians, positive to camera left."""
        return -np.arctan(self.horizontal_rays(pixels))

    def field_of_view(self):
        """Return the horizontal view limits."""
        row = float(np.clip(self.matrix[1, 2], 0, self.height - 1))
        angles = self.bearings([(0, row), (self.width, row)])
        return float(min(angles)), float(max(angles))

    def project(self, points):
        """Project optical-frame XYZ points, including raw camera distortion."""
        pixels, _ = cv2.projectPoints(np.asarray(points, dtype=float), np.zeros(3),
                                      np.zeros(3), self.matrix, self.distortion)
        return pixels[:, 0, :]


@dataclass
class DetectorSettings:
    """Color, physical size, and valid sensor range limits (meters)."""

    value_max: float = 70.0
    saturation_max: float = 120.0
    min_area: float = 100.0
    min_width: int = 6
    min_height: int = 8
    min_fill: float = 0.30
    width_min_m: float = 0.23
    width_max_m: float = 0.30
    range_min: float = 0.15
    range_max: float = 5.0
    min_depth_pixels: int = 15
    min_lidar_points: int = 3
    cluster_tolerance_m: float = 0.12
    min_surface_fraction: float = 0.20
    min_depth_valid_fraction: float = 0.04
    max_surface_std_m: float = 0.18
    width_tolerance_m: float = 0.015
    range_window: int = 3
    range_track_timeout_s: float = 0.6
    range_jump_m: float = 0.20
    depth_ema_tau_s: float = 0.35
    # Merge detections that are pieces of one object (e.g. a rover's deck and track):
    # overlapping horizontally, close vertically, at about the same distance.
    # merge_max_gap_px = 0 disables merging.
    merge_max_gap_px: float = 0.0
    merge_min_overlap: float = 0.5
    merge_max_range_diff_m: float = 0.3


def validate(settings):
    """Reject invalid settings at startup and during parameter changes."""
    if not all(math.isfinite(value) for value in asdict(settings).values()):
        raise ValueError('All detector settings must be finite')
    if not (0 < settings.value_max <= 255 and 0 <= settings.saturation_max <= 255
            and settings.min_area > 0 and settings.min_width > 0 and settings.min_height > 0
            and 0 < settings.min_fill <= 1
            and 0 < settings.width_min_m <= settings.width_max_m
            and 0 < settings.range_min < settings.range_max
            and settings.min_depth_pixels > 0 and settings.min_lidar_points > 0
            and settings.cluster_tolerance_m > 0
            and 0 < settings.min_surface_fraction <= 1
            and 0 < settings.min_depth_valid_fraction <= 1
            and settings.max_surface_std_m > 0 and 0 <= settings.width_tolerance_m <= 0.03
            and 1 <= settings.range_window <= 9
            and settings.range_track_timeout_s > 0 and settings.range_jump_m > 0
            and 0 <= settings.depth_ema_tau_s <= 2.0
            and settings.merge_max_gap_px >= 0 and 0 < settings.merge_min_overlap <= 1
            and settings.merge_max_range_diff_m >= 0):
        raise ValueError('Invalid color, blob, width, range, or sample limits')


@dataclass
class Blob:
    """A color candidate before its distance or physical width is known."""

    bbox: tuple
    contour: np.ndarray
    bearing: float
    angle_min: float
    angle_max: float
    ray_span: float
    area_px: float
    clipped: bool

    def interval(self):
        """Return an unresolved angular interval for downstream consumers."""
        return {'angle_min': self.angle_min, 'angle_max': self.angle_max}


@dataclass
class Detection:
    """A measured silhouette; distance is horizontal radial range, not optical Z."""

    bbox: tuple
    bearing: float
    angle_min: float
    angle_max: float
    distance_m: float
    distance_source: str
    confidence: float
    area_px: float
    width_m: float = 0.0
    axial_m: float = 0.0
    # Informational: measured width is in the target band (23-33 cm) and fully visible.
    # Every Detection is an obstacle regardless of this flag.
    accepted: bool = True
    width_std_m: float = 0.0
    range_std_m: float = 0.0
    sample_count: int = 0
    nominal_width: bool = True
    # Cut off by the left/right image border: width_m is only a lower bound, so the
    # width can never be confirmed to be in the target band (accepted stays False).
    clipped: bool = False
    # For a merged detection: the image boxes of the blobs it was merged from
    # (bbox is then their union). None for a single blob.
    parts: tuple = None

    def to_dict(self):
        """Serialize only finite metric measurements."""
        return asdict(self)


def find_black_blobs(bgr, camera, settings):
    """Segment near-black connected silhouettes without assuming their size."""
    if bgr.shape[:2] != (camera.height, camera.width):
        raise ValueError('Image and camera calibration sizes differ')
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 0, 0), (179, settings.saturation_max, settings.value_max))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    blobs = []
    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, width, height = cv2.boundingRect(contour)
        if (area < settings.min_area or width < settings.min_width
                or height < settings.min_height or area / (width * height) < settings.min_fill):
            continue
        rays = camera.horizontal_rays([(x, y + height / 2),
                                       (x + width, y + height / 2),
                                       (x + width / 2, y + height / 2)])
        angles = -np.arctan(rays)
        if not np.isfinite(angles).all():
            continue
        ray_span = abs(float(rays[1] - rays[0]))
        # A tiny color speck cannot be a target-width object even at maximum range.
        if ray_span * settings.range_max < settings.width_min_m - settings.width_tolerance_m:
            continue
        # Only the left/right border can cut off the (horizontal) width; touching the
        # top or bottom border leaves the measured width complete.
        clipped = x <= 0 or x + width >= camera.width
        blobs.append(Blob((x, y, width, height), contour, float(angles[2]),
                          float(min(angles[:2])), float(max(angles[:2])),
                          ray_span, float(area), clipped))
    return blobs, mask


@dataclass
class RangeEstimate:
    """Supported optical-Z measurement and robust spread of its contributing samples."""

    axial_m: float = math.nan
    std_m: float = 0.0
    samples: int = 0
    support_fraction: float = 0.0
    raw_axial_m: float = math.nan
    filter_method: str = 'none'
    ema_alpha: float = 1.0


def measure_blob(blob, axial_m, source, settings, estimate=None):
    """Compute width = optical Z * (right X/Z - left X/Z)."""
    # A clipped blob keeps its measured distance (it is a detection), but its width is
    # only a lower bound, so it is never marked as target-sized (accepted).
    if not math.isfinite(axial_m) or axial_m <= 0:
        return None
    distance = axial_m / math.cos(blob.bearing)
    if not settings.range_min <= distance <= settings.range_max:
        return None
    width = axial_m * blob.ray_span
    nominal = settings.width_min_m - 1e-7 <= width <= settings.width_max_m + 1e-7
    width_std = (math.hypot(blob.ray_span * estimate.std_m, 0.005)
                 if estimate is not None else 0.0)
    tolerance = min(settings.width_tolerance_m, 2 * width_std)
    accepted = not blob.clipped and settings.width_min_m - tolerance - 1e-7 <= width <= (
        settings.width_max_m + tolerance + 1e-7)
    confidence = (max(0.2, 1 - estimate.std_m / settings.max_surface_std_m)
                  if estimate is not None else 1.0)
    return Detection(blob.bbox, blob.bearing, blob.angle_min, blob.angle_max,
                     distance, source, confidence, blob.area_px, width, axial_m, accepted,
                     width_std, estimate.std_m / math.cos(blob.bearing) if estimate else 0.0,
                     estimate.samples if estimate else 0, nominal and not blob.clipped,
                     blob.clipped)


def mergeable(first, second, settings):
    """Return whether two detections look like pieces of one object."""
    ax, ay, aw, ah = first.bbox
    bx, by, bw, bh = second.bbox
    overlap = min(ax + aw, bx + bw) - max(ax, bx)
    if overlap < settings.merge_min_overlap * min(aw, bw):
        return False
    gap = max(ay, by) - min(ay + ah, by + bh)   # negative when they overlap vertically
    if gap > max(settings.merge_max_gap_px, 0.1 * max(ah, bh)):
        return False
    return abs(first.distance_m - second.distance_m) <= settings.merge_max_range_diff_m


def merge_pair(first, second, settings):
    """Combine two detection pieces into one; distance from the nearer piece."""
    near = first if first.distance_m <= second.distance_m else second
    angle_min = min(first.angle_min, second.angle_min)
    angle_max = max(first.angle_max, second.angle_max)
    bearing = (angle_min + angle_max) / 2
    # Angles are -atan(X/Z), so the optical X/Z span is the tangent difference.
    width = near.axial_m * abs(math.tan(angle_max) - math.tan(angle_min))
    width_std = max(first.width_std_m, second.width_std_m)
    clipped = first.clipped or second.clipped
    tolerance = min(settings.width_tolerance_m, 2 * width_std)
    accepted = not clipped and settings.width_min_m - tolerance - 1e-7 <= width <= (
        settings.width_max_m + tolerance + 1e-7)
    nominal = (not clipped
               and settings.width_min_m - 1e-7 <= width <= settings.width_max_m + 1e-7)
    x = min(first.bbox[0], second.bbox[0])
    y = min(first.bbox[1], second.bbox[1])
    right = max(first.bbox[0] + first.bbox[2], second.bbox[0] + second.bbox[2])
    bottom = max(first.bbox[1] + first.bbox[3], second.bbox[1] + second.bbox[3])
    parts = (first.parts or (first.bbox,)) + (second.parts or (second.bbox,))
    return Detection((x, y, right - x, bottom - y), bearing, angle_min, angle_max,
                     near.distance_m, near.distance_source,
                     min(first.confidence, second.confidence),
                     first.area_px + second.area_px, width, near.axial_m, accepted, width_std,
                     max(first.range_std_m, second.range_std_m),
                     first.sample_count + second.sample_count, nominal, clipped, parts)


def merge_detections(items, settings):
    """Repeatedly merge detection pieces of one object (see mergeable)."""
    if settings.merge_max_gap_px <= 0:
        return list(items)
    items = list(items)
    merged = True
    while merged:
        merged = False
        for i in range(len(items)):
            for j in range(i + 1, len(items)):
                if mergeable(items[i], items[j], settings):
                    items[i] = merge_pair(items[i], items[j], settings)
                    del items[j]
                    merged = True
                    break
            if merged:
                break
    return items


def depth_for_blob(blob, depth_m, settings):
    """Sample the eroded silhouette, excluding invalid depth and background."""
    return depth_estimate_for_blob(blob, depth_m, settings).axial_m


def depth_estimate_for_blob(blob, depth_m, settings):
    """Require spatial support and robust spread before reporting blob depth."""
    x, y, width, height = blob.bbox
    local = np.zeros((height, width), np.uint8)
    contour = blob.contour - np.array([[[x, y]]], dtype=blob.contour.dtype)
    cv2.drawContours(local, [contour], -1, 255, cv2.FILLED)
    local = cv2.erode(local, np.ones((3, 3), np.uint8))
    patch = depth_m[y:y + height, x:x + width]
    if patch.shape != local.shape:
        raise ValueError('Depth must be aligned with the RGB image')
    values = patch[(local != 0) & np.isfinite(patch)
                   & (patch >= settings.range_min) & (patch <= settings.range_max)]
    minimum = max(settings.min_depth_pixels,
                  math.ceil(np.count_nonzero(local) * settings.min_depth_valid_fraction))
    return estimate_surface(values, minimum, settings, noise_floor=0.01)


def robust_surface(values, minimum, tolerance):
    """Use the nearest supported depth cluster; never average foreground and background."""
    settings = DetectorSettings(cluster_tolerance_m=tolerance)
    return estimate_surface(values, minimum, settings).axial_m


def estimate_surface(values, minimum, settings, noise_floor=0.01):
    """Reject minority speckles and broad/bridged surfaces using support and MAD."""
    values = np.sort(np.asarray(values)[np.isfinite(values)])
    if len(values) < minimum:
        return RangeEstimate()
    minimum = max(minimum, math.ceil(settings.min_surface_fraction * len(values)))
    clusters = np.split(values, np.flatnonzero(
        np.diff(values) > settings.cluster_tolerance_m) + 1)
    for cluster in clusters:
        if len(cluster) >= minimum:
            median = float(np.median(cluster))
            spread = 1.4826 * float(np.median(np.abs(cluster - median)))
            if spread > settings.max_surface_std_m:
                continue
            trimmed = cluster[np.abs(cluster - median) <= max(3 * spread, noise_floor * 3)]
            if len(trimmed) >= minimum:
                return RangeEstimate(float(np.median(trimmed)), max(spread, noise_floor),
                                     len(trimmed), len(trimmed) / len(values))
    return RangeEstimate()


def scan_points_in_camera(scan, rotation, translation):
    """Transform valid LaserScan points from the laser frame to the optical frame."""
    ranges = np.asarray(scan.ranges, dtype=float)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    points = np.column_stack((ranges[valid] * np.cos(angles[valid]),
                              ranges[valid] * np.sin(angles[valid]), np.zeros(valid.sum())))
    return points @ np.asarray(rotation).T + np.asarray(translation)


def lidar_for_blobs(blobs, points, camera, settings):
    """Associate scan returns with the actual image contour, including its vertical extent."""
    return [item.axial_m for item in lidar_estimates_for_blobs(blobs, points, camera, settings)]


def lidar_estimates_for_blobs(blobs, points, camera, settings):
    """Return supported scan-cluster estimates with their uncertainty."""
    points = np.asarray(points).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1) & (points[:, 2] > 0)]
    if not len(points):
        return [RangeEstimate() for _ in blobs]
    pixels = camera.project(points)
    inside = ((pixels[:, 0] >= 0) & (pixels[:, 0] < camera.width)
              & (pixels[:, 1] >= 0) & (pixels[:, 1] < camera.height))
    points, pixels = points[inside], pixels[inside]
    result = []
    for blob in blobs:
        selected = [index for index, pixel in enumerate(pixels)
                    if cv2.pointPolygonTest(blob.contour, tuple(map(float, pixel)), False) >= 0]
        values = points[selected, 2]
        result.append(estimate_surface(values, settings.min_lidar_points, settings,
                                       noise_floor=0.01))
    return result
