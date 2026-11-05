"""Testable black-object detection and calibrated camera-bearing estimates."""

from dataclasses import asdict, dataclass
import math

import cv2
import numpy as np


@dataclass
class CameraModel:
    """Camera calibration at the resolution being processed."""

    width: int
    height: int
    matrix: np.ndarray
    distortion: np.ndarray

    @classmethod
    def from_info(cls, info, width, height, rectified):
        if info.width <= 0 or info.height <= 0:
            raise ValueError('CameraInfo has no image dimensions')
        if rectified:
            matrix = np.asarray(info.p, dtype=float).reshape(3, 4)[:, :3].copy()
            distortion = np.zeros(5)
        else:
            matrix = np.asarray(info.k, dtype=float).reshape(3, 3).copy()
            if info.distortion_model not in ('plumb_bob', 'rational_polynomial', ''):
                raise ValueError('Unsupported raw distortion model: ' + info.distortion_model)
            distortion = np.asarray(info.d, dtype=float)
        matrix[0, :] *= width / info.width
        matrix[1, :] *= height / info.height
        if not np.isfinite(matrix).all() or matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
            raise ValueError('CameraInfo has invalid focal lengths')
        return cls(width, height, matrix, distortion)

    def horizontal_rays(self, pixels):
        points = np.asarray(pixels, dtype=np.float32).reshape(-1, 1, 2)
        return cv2.undistortPoints(points, self.matrix, self.distortion)[:, 0, 0]

    def bearings(self, pixels):
        # Optical image x grows right; positive ROS bearing is left.
        return -np.arctan(self.horizontal_rays(pixels))

    def field_of_view(self):
        row = float(np.clip(self.matrix[1, 2], 0, self.height - 1))
        angles = self.bearings([(0, row), (self.width - 1, row)])
        return float(min(angles)), float(max(angles))


@dataclass
class DetectorSettings:
    """Parameters for a deliberately simple Lab 1 color baseline."""

    value_max: float = 105.0
    saturation_max: float = 160.0
    dark_ratio: float = 0.70
    min_background_value: float = 90.0
    min_area: float = 180.0
    min_width: int = 10
    min_height: int = 12
    min_fill: float = 0.30
    min_bottom_fraction: float = 0.50
    roi_top: float = 0.15
    roi_bottom: float = 0.95
    side_top: float = 0.05
    side_bottom: float = 0.12
    object_width_m: float = 0.27
    range_min: float = 0.15
    range_max: float = 5.0
    min_depth_pixels: int = 15


@dataclass
class Detection:
    """A candidate obstacle, with explicit distance provenance."""

    bbox: tuple
    bearing: float
    angle_min: float
    angle_max: float
    distance_m: float
    distance_source: str
    confidence: float
    area_px: float

    def to_dict(self):
        record = asdict(self)
        record['bearing_deg'] = math.degrees(self.bearing)
        record['angle_min_deg'] = math.degrees(self.angle_min)
        record['angle_max_deg'] = math.degrees(self.angle_max)
        return record


def make_roi(height, width, settings):
    """Exclude the sky and dark outer image/body corners."""
    points = np.array([
        [settings.side_top * width, settings.roi_top * height],
        [(1 - settings.side_top) * width, settings.roi_top * height],
        [(1 - settings.side_bottom) * width, settings.roi_bottom * height],
        [settings.side_bottom * width, settings.roi_bottom * height],
    ], dtype=np.int32)
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(mask, [points], 255)
    return mask, points


def detect_black_objects(bgr, camera, settings, depth_m=None):
    """Return detections, a black-pixel mask, ROI, and the applied V threshold."""
    height, width = bgr.shape[:2]
    if (width, height) != (camera.width, camera.height):
        raise ValueError('Image and camera calibration sizes differ')
    if depth_m is not None and depth_m.shape != (height, width):
        raise ValueError('Depth must be aligned with the RGB image')
    roi, polygon = make_roi(height, width, settings)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    background = max(settings.min_background_value,
                     float(np.percentile(hsv[:, :, 2][roi != 0], 75)))
    threshold = min(settings.value_max, settings.dark_ratio * background)
    black = ((hsv[:, :, 2] <= threshold)
             & (hsv[:, :, 1] <= settings.saturation_max)
             & (roi != 0)).astype(np.uint8) * 255
    black = cv2.morphologyEx(black, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    black = cv2.morphologyEx(black, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    black = cv2.bitwise_and(black, roi)
    contours = cv2.findContours(black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    detections = []
    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, box_width, box_height = cv2.boundingRect(contour)
        fill = area / (box_width * box_height)
        if (area < settings.min_area or box_width < settings.min_width
                or box_height < settings.min_height or fill < settings.min_fill
                or y + box_height < settings.min_bottom_fraction * height
                or max(box_width / box_height, box_height / box_width) > 4.5):
            continue
        # Use the whole angular silhouette, not just its center bin.
        pixels = [(x, y + box_height / 2),
                  (x + box_width, y + box_height / 2),
                  (x + box_width / 2, y + box_height / 2)]
        rays = camera.horizontal_rays(pixels)
        angles = -np.arctan(rays)
        if not np.isfinite(angles).all() or abs(rays[1] - rays[0]) < 1e-6:
            continue
        bearing = float(angles[2])
        # Approximate axial distance from the handout's known obstacle width.
        axial = settings.object_width_m / abs(float(rays[1] - rays[0]))
        source = 'size_estimate'
        if depth_m is not None:
            local = np.zeros((box_height, box_width), dtype=np.uint8)
            shifted = contour - np.array([[[x, y]]], dtype=contour.dtype)
            cv2.drawContours(local, [shifted], -1, 255, cv2.FILLED)
            local = cv2.erode(local, np.ones((3, 3), np.uint8))
            patch = depth_m[y:y + box_height, x:x + box_width]
            valid = ((local != 0) & np.isfinite(patch)
                     & (patch >= settings.range_min) & (patch <= settings.range_max))
            values = patch[valid]
            if values.size >= settings.min_depth_pixels:
                axial = float(np.percentile(values, 30))
                source = 'depth'
        distance = axial / max(0.1, math.cos(bearing))
        confidence = min(1.0, area / (settings.min_area * 3)) * min(1.0, fill / 0.65)
        if source == 'size_estimate':
            confidence *= 0.75
        detections.append(Detection(
            (x, y, box_width, box_height), bearing,
            float(min(angles[:2])), float(max(angles[:2])),
            float(distance), source, float(confidence), float(area)))
    detections.sort(key=lambda item: item.distance_m)
    return detections, black, polygon, threshold
