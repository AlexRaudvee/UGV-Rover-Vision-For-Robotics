"""Marker detection: preprocessing, multi-family detection and merging (no ROS imports).

Pipeline (assignment Fig. 1b): image -> preprocessing -> detect marker + category.
Every reported corner is in the ORIGINAL image's pixel coordinates, whatever
preprocessing (undistortion, upscaling) produced it, so detections of different
pipelines can be compared and later used for pose estimation with the raw K and D.
"""

from dataclasses import dataclass, replace
import time

import cv2
import numpy as np

from . import aruco_compat as compat

# Category = OpenCV dictionary name without the DICT_ prefix. Trim this to the
# families actually used in the lab: every extra family costs a detector pass.
DEFAULT_FAMILIES = ('APRILTAG_36h11', 'APRILTAG_25h9', '4X4_50', '5X5_100', '6X6_250')


@dataclass(frozen=True)
class PipelineConfig:
    """Which preprocessing steps and detector settings to use."""

    families: tuple = DEFAULT_FAMILIES
    undistort: bool = False      # remove lens distortion first (needs a CameraModel)
    clahe: bool = False          # local contrast equalization (backlight, shadows, glare)
    sharpen: bool = False        # unsharp mask (mild defocus / motion blur)
    upscale: float = 1.0         # enlarge before detection (small / distant markers)
    tuned: bool = False          # detector parameters tuned for small, blurred, skewed tags
    fuse_raw: bool = False       # also detect on the unprocessed image and merge results
    clahe_clip: float = 2.0
    clahe_tile: int = 8
    sharpen_sigma: float = 1.5
    sharpen_amount: float = 1.0

    @property
    def steps(self):
        names = [('undistort', self.undistort), ('clahe', self.clahe),
                 ('sharpen', self.sharpen), (f'x{self.upscale:g}', self.upscale != 1.0)]
        return [name for name, enabled in names if enabled]


PRESETS = {
    'baseline': PipelineConfig(),
    'clahe': PipelineConfig(clahe=True),
    'sharpen': PipelineConfig(sharpen=True),
    'upscale': PipelineConfig(upscale=2.0),
    'tuned': PipelineConfig(tuned=True),
    'undistort': PipelineConfig(undistort=True),
    'improved': PipelineConfig(clahe=True, upscale=2.0, tuned=True, fuse_raw=True),
}


@dataclass
class Detection:
    """One marker: category (family), id and corners in original-image pixels.

    Corner order is OpenCV's: the marker's own top-left, top-right, bottom-right,
    bottom-left, so corner 0 shows the marker orientation.
    """

    family: str
    marker_id: int
    corners: np.ndarray
    variant: str = 'raw'

    @property
    def center(self):
        return self.corners.mean(axis=0)

    @property
    def side_px(self):
        return float(np.linalg.norm(self.corners - np.roll(self.corners, -1, axis=0),
                                    axis=1).mean())

    def as_dict(self):
        return {'family': self.family, 'id': self.marker_id, 'variant': self.variant,
                'center': [round(float(v), 2) for v in self.center],
                'side_px': round(self.side_px, 2),
                'corners': [[round(float(x), 2), round(float(y), 2)] for x, y in self.corners]}


@dataclass
class DetectionResult:
    detections: list
    runtime_ms: float
    pipeline: str


# ---------------------------------------------------------------- preprocessing
def to_gray(image):
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image


def apply_clahe(gray, clip=2.0, tile=8):
    return cv2.createCLAHE(clipLimit=clip, tileGridSize=(tile, tile)).apply(gray)


def unsharp(gray, sigma=1.5, amount=1.0):
    blurred = cv2.GaussianBlur(gray, (0, 0), sigma)
    return cv2.addWeighted(gray, 1.0 + amount, blurred, -amount, 0)


def resize(gray, factor):
    return cv2.resize(gray, None, fx=factor, fy=factor, interpolation=cv2.INTER_CUBIC)


def detector_parameters(tuned):
    """Default OpenCV parameters, or a set tuned for small, blurred and skewed tags."""
    parameters = compat.detector_parameters()
    if tuned:
        # More threshold windows: tags of many sizes and uneven lighting in one frame.
        parameters.adaptiveThreshWinSizeMin = 3
        parameters.adaptiveThreshWinSizeMax = 53
        parameters.adaptiveThreshWinSizeStep = 4
        # Accept smaller candidate contours (distant tags); default 0.03.
        parameters.minMarkerPerimeterRate = 0.01
        # Looser polygon fit for blurred or lens-curved edges; default 0.03.
        parameters.polygonalApproxAccuracyRate = 0.05
        # Allow tags close to the image border; default 3 px.
        parameters.minDistanceToBorder = 1
        # Sub-pixel corners: matters for distance estimation in Session 2.
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return parameters


# ---------------------------------------------------------------- detector
class MarkerDetector:
    """Detect markers of several families with a configurable preprocessing pipeline."""

    def __init__(self, config=PipelineConfig(), camera=None):
        if config.undistort and camera is None:
            raise ValueError('undistort needs a CameraModel (camera_info)')
        families = tuple(compat.normalize_family(name) for name in config.families)
        if not families:
            raise ValueError('At least one marker family is required')
        self.config = replace(config, families=families)
        self.camera = camera
        parameters = detector_parameters(config.tuned)
        self._detectors = [(family, compat.make_detector(compat.dictionary(family), parameters))
                           for family in families]

    @property
    def name(self):
        return '+'.join(self.config.steps + (['tuned'] if self.config.tuned else [])) or 'raw'

    def _variants(self, gray):
        """(name, image, scale, undistorted) for each image the detectors run on."""
        cfg = self.config
        work = gray
        if cfg.undistort:
            work = self.camera.undistort(work)
        if cfg.clahe:
            work = apply_clahe(work, cfg.clahe_clip, cfg.clahe_tile)
        if cfg.sharpen:
            work = unsharp(work, cfg.sharpen_sigma, cfg.sharpen_amount)
        if cfg.upscale != 1.0:
            work = resize(work, cfg.upscale)
        processed = ('+'.join(cfg.steps) or 'raw', work, cfg.upscale, cfg.undistort)
        if cfg.fuse_raw and cfg.steps:
            return [('raw', gray, 1.0, False), processed]
        return [processed]

    def detect(self, image):
        start = time.perf_counter()
        gray = to_gray(image)
        size = (gray.shape[1], gray.shape[0])
        found = []
        for variant_index, (variant, work, scale, undistorted) in enumerate(self._variants(gray)):
            for family_index, (family, detect) in enumerate(self._detectors):
                corners, ids = detect(work)
                if ids is None:
                    continue
                for quad, marker_id in zip(corners, ids.ravel()):
                    points = quad.reshape(4, 2).astype(np.float32) / scale
                    if undistorted:
                        points = self.camera.distort_points(points, size)
                    found.append(((family_index, variant_index),
                                  Detection(family, int(marker_id), points, variant)))
        detections = merge_duplicates(found)
        return DetectionResult(detections, (time.perf_counter() - start) * 1000.0, self.name)


def merge_duplicates(found):
    """Keep one detection per physical marker.

    The same marker can be found on several image variants, and occasionally a
    quad decodes under two families. Candidates are ranked by (family order,
    variant order); a candidate whose centre lies within half a side length of
    an already kept marker is the same physical marker and is dropped.
    """
    kept = []
    for _, candidate in sorted(found, key=lambda item: item[0]):
        duplicate = any(np.linalg.norm(candidate.center - other.center)
                        < 0.5 * min(candidate.side_px, other.side_px) for other in kept)
        if not duplicate:
            kept.append(candidate)
    return sorted(kept, key=lambda d: (d.family, d.marker_id, float(d.center[0])))


def short_family(family):
    return family[len('APRILTAG_'):] if family.startswith('APRILTAG_') else family


def draw(image, detections, status=''):
    """Annotated BGR copy: outline, orientation corner, 'family:id' label."""
    canvas = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if image.ndim == 2 else image.copy()
    for detection in detections:
        quad = detection.corners.round().astype(np.int32)
        cv2.polylines(canvas, [quad], True, (40, 220, 40), 2, cv2.LINE_AA)
        cv2.circle(canvas, tuple(int(v) for v in quad[0]), 4, (0, 0, 255), -1, cv2.LINE_AA)
        text = f'{short_family(detection.family)}:{detection.marker_id}'
        x, y = quad[:, 0].min(), max(14, quad[:, 1].min() - 6)
        cv2.putText(canvas, text, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(canvas, text, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (60, 255, 255), 1, cv2.LINE_AA)
    if status:
        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 22), (30, 27, 24), -1)
        cv2.putText(canvas, status[:100], (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (235, 235, 235), 1, cv2.LINE_AA)
    return canvas


def result_dict(result):
    """JSON-ready summary of a DetectionResult."""
    return {'pipeline': result.pipeline, 'runtime_ms': round(result.runtime_ms, 2),
            'count': len(result.detections),
            'detections': [d.as_dict() for d in result.detections]}


__all__ = ['DEFAULT_FAMILIES', 'PRESETS', 'PipelineConfig', 'Detection', 'DetectionResult',
           'MarkerDetector', 'draw', 'result_dict', 'merge_duplicates']
