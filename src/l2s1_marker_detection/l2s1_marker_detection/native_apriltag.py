"""Adapter for the Python extension built by the bundled AprilTag CMake package."""

import importlib
import importlib.util
import os
from pathlib import Path
import sys

import numpy as np


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
        if not results:
            return [], None
        # Native order: lower-left, lower-right, upper-right, upper-left.
        # Match OpenCV: upper-left, upper-right, lower-right, lower-left.
        corners = [np.asarray(item['lb-rb-rt-lt'], np.float32)[[3, 2, 1, 0]]
                   .reshape(1, 4, 2) for item in results]
        ids = np.array([[item['id']] for item in results], dtype=np.int32)
        return corners, ids

    return detect
