"""Pinhole + plumb_bob camera model loaded from a ROS camera_info YAML or message."""

from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class CameraModel:
    """Intrinsics K (3x3) and distortion D for a calibration image size."""

    K: np.ndarray
    D: np.ndarray
    width: int
    height: int
    _maps: dict = field(default_factory=dict, repr=False, compare=False)

    @classmethod
    def from_yaml(cls, path, rectified=False):
        import yaml
        with open(path) as stream:
            data = yaml.safe_load(stream)
        matrix = np.array(data['camera_matrix']['data'], float).reshape(3, 3)
        distortion = np.array(data['distortion_coefficients']['data'], float)
        if rectified:
            matrix = np.array(data['projection_matrix']['data'], float).reshape(3, 4)[:, :3]
            distortion = np.zeros(5)
        return cls(matrix, distortion,
                   int(data['image_width']), int(data['image_height']))

    @classmethod
    def from_camera_info(cls, message, rectified=False):
        """Build from a sensor_msgs/CameraInfo (duck-typed, no ROS import)."""
        matrix = np.array(message.p, float).reshape(3, 4)[:, :3] if rectified else np.array(message.k, float).reshape(3, 3)
        distortion = np.zeros(5) if rectified else np.array(message.d, float)
        return cls(matrix, distortion,
                   int(message.width), int(message.height))

    def matrix_for(self, width, height):
        """K rescaled when frames arrive at a different resolution than calibrated."""
        scaled = self.K.copy()
        scaled[0] *= width / self.width
        scaled[1] *= height / self.height
        return scaled

    def undistort(self, image):
        """Remove lens distortion, keeping the same K so pixel scale is unchanged."""
        height, width = image.shape[:2]
        if (width, height) not in self._maps:
            matrix = self.matrix_for(width, height)
            self._maps[(width, height)] = cv2.initUndistortRectifyMap(
                matrix, self.D, None, matrix, (width, height), cv2.CV_16SC2)
        map1, map2 = self._maps[(width, height)]
        return cv2.remap(image, map1, map2, cv2.INTER_LINEAR)

    def distort_points(self, points, image_size):
        """Map pixels of an ``undistort`` image back to the original (raw) image."""
        width, height = image_size
        matrix = self.matrix_for(width, height)
        points = np.asarray(points, float).reshape(-1, 2)
        normalized = np.column_stack([(points[:, 0] - matrix[0, 2]) / matrix[0, 0],
                                      (points[:, 1] - matrix[1, 2]) / matrix[1, 1],
                                      np.ones(len(points))])
        projected, _ = cv2.projectPoints(normalized, np.zeros(3), np.zeros(3), matrix, self.D)
        return projected.reshape(-1, 2).astype(np.float32)
