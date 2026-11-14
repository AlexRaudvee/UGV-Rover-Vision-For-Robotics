"""Adapter for the Python extension built by the bundled AprilTag CMake package."""

import importlib
import importlib.util
import os
from pathlib import Path
import sys

import numpy as np
import cv2


def _load_extension():
    # A normal import also supports laptop builds exposed through PYTHONPATH.
    try:
        module = importlib.import_module('apriltag')
        if hasattr(module, 'apriltag'):
            return module
    except ImportError:
        pass
    # Plain CMake packages do not always add site-packages to PYTHONPATH.
    # Search the active colcon prefixes, including an isolated apriltag prefix.
    prefixes = set()
    for variable in ('AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH'):
        for value in os.environ.get(variable, '').split(os.pathsep):
            if value:
                prefix = Path(value)
                prefixes.update((prefix, prefix / 'apriltag', prefix.parent / 'apriltag'))
    version = f'python{sys.version_info.major}.{sys.version_info.minor}'
    for prefix in sorted(prefixes):
        directory = prefix / 'lib' / version / 'site-packages'
        for path in sorted(directory.glob('apriltag.*.so')):
            spec = importlib.util.spec_from_file_location('apriltag', path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module
    raise ImportError(
        'tagStandard41h12 needs the bundled AprilTag Python extension. '
        'Build with: colcon build --packages-select apriltag '
        '--cmake-args -DBUILD_PYTHON_WRAPPER=ON -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF; then source install/setup.bash. '
        'The build needs python3-dev and python3-numpy.')


def make_detector():
    detector = _load_extension().apriltag('tagStandard41h12', decimate=1.0)

    def detect(gray):
        results = detector.detect(np.ascontiguousarray(gray, dtype=np.uint8))
        detect.qualities = [{'hamming': int(item['hamming']),
                             'decision_margin': float(item['margin'])} for item in results]
        if not results:
            return [], None
        # Native order: lower-left, lower-right, upper-right, upper-left.
        # Match OpenCV: upper-left, upper-right, lower-right, lower-left.
        corners = [np.asarray(item['lb-rb-rt-lt'], np.float32)[[3, 2, 1, 0]]
                   .reshape(1, 4, 2) for item in results]
        ids = np.array([[item['id']] for item in results], dtype=np.int32)
        return corners, ids

    return detect


def estimate_pose(corners, tag_size_m, camera_matrix, distortion=None):
    """AprilTag 3 pose from TL,TR,BR,BL corners; size is the detected square."""
    module = _load_extension()
    if not hasattr(module, 'estimate_pose'):
        raise ImportError('Rebuild apriltag to enable its estimate_pose binding')
    points = np.asarray(corners, np.float64).reshape(4, 2)
    matrix = np.asarray(camera_matrix, np.float64).reshape(3, 3)
    if not np.all(np.isfinite(points)) or not cv2.isContourConvex(points.astype(np.float32)):
        raise ValueError('tag corners must form a finite convex square')
    if abs(cv2.contourArea(points.astype(np.float32))) < 4:
        raise ValueError('tag is too small or degenerate for pose estimation')
    if distortion is not None and np.any(distortion):
        points = cv2.undistortPoints(points.reshape(4, 1, 2), matrix,
                                     np.asarray(distortion, np.float64), P=matrix).reshape(4, 2)
    native = points[[3, 2, 1, 0]]
    ideal = np.array([[-1, 1], [1, 1], [1, -1], [-1, -1]], np.float32)
    homography = cv2.getPerspectiveTransform(ideal, native.astype(np.float32))
    result = module.estimate_pose(native, homography, float(tag_size_m),
                                  matrix[0, 0], matrix[1, 1], matrix[0, 2], matrix[1, 2])
    object_points = np.column_stack((ideal * (float(tag_size_m)/2), np.zeros(4)))
    rotation_vector, _ = cv2.Rodrigues(result['rotation'])
    projected, _ = cv2.projectPoints(object_points, rotation_vector, result['translation'],
                                     matrix, np.zeros(5))
    result['reprojection_error_px'] = float(np.sqrt(np.mean(
        np.sum((projected.reshape(4, 2) - native)**2, axis=1))))
    if not np.all(np.isfinite(result['rotation'])) or not np.isfinite(result['reprojection_error_px']):
        raise ValueError('AprilTag pose contains nonfinite values')
    return result
