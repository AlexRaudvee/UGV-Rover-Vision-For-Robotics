"""One interface over the OpenCV ArUco/AprilTag API before and after OpenCV 4.7.

The rover's Ubuntu 22.04 OpenCV (4.5.4) uses ``cv2.aruco.detectMarkers`` and
``DetectorParameters_create``; OpenCV >= 4.7 uses ``ArucoDetector`` objects.
"""

import cv2

_ARUCO = getattr(cv2, 'aruco', None)
_NEW_API = _ARUCO is not None and hasattr(_ARUCO, 'ArucoDetector')


def _aruco():
    if _ARUCO is None:
        raise ImportError('This OpenCV build has no aruco module. Install python3-opencv '
                          '(Ubuntu) or opencv-contrib-python.')
    return _ARUCO


def normalize_family(name):
    """Map 'tag36h11', 'apriltag_36H11', '6x6_250' to OpenCV's spelling."""
    aruco = _aruco()
    text = name.strip()
    if hasattr(aruco, 'DICT_' + text):
        return text
    if text.lower().startswith('tag'):
        text = 'APRILTAG_' + text[3:]
    if text.upper().startswith('APRILTAG_'):
        candidate = 'APRILTAG_' + text[9:].lower()
    else:
        candidate = text.upper()
    if not hasattr(aruco, 'DICT_' + candidate):
        raise ValueError(f'Unknown marker family {name!r}')
    return candidate


def dictionary(family):
    """Return the predefined dictionary for a normalized family name."""
    aruco = _aruco()
    constant = getattr(aruco, 'DICT_' + normalize_family(family))
    if hasattr(aruco, 'getPredefinedDictionary'):
        return aruco.getPredefinedDictionary(constant)
    return aruco.Dictionary_get(constant)


def detector_parameters():
    """Return default detector parameters."""
    aruco = _aruco()
    return aruco.DetectorParameters() if _NEW_API else aruco.DetectorParameters_create()


def make_detector(marker_dictionary, parameters):
    """Return ``detect(gray) -> (corners, ids)``; ids is None when nothing is found."""
    aruco = _aruco()
    if _NEW_API:
        detector = aruco.ArucoDetector(marker_dictionary, parameters)
        return lambda gray: detector.detectMarkers(gray)[:2]
    return lambda gray: aruco.detectMarkers(gray, marker_dictionary, parameters=parameters)[:2]


def marker_image(family, marker_id, side_px):
    """Render a marker (black border included) as a square uint8 image."""
    aruco = _aruco()
    marker_dictionary = dictionary(family)
    if hasattr(aruco, 'generateImageMarker'):
        return aruco.generateImageMarker(marker_dictionary, marker_id, side_px)
    return aruco.drawMarker(marker_dictionary, marker_id, side_px)
