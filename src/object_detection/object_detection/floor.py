"""
Floor plane from stereo depth geometry, independent of colour and lighting.

The floor is a plane ``height(p) = normal . p + offset`` in the camera optical frame
(x right, y down, z forward), with ``normal`` pointing up, away from the floor.
``offset`` is then the camera's height above the floor.

Each frame the plane is fitted to the depth points with RANSAC, constrained to stay
near a prior plane from the robot's TF (camera mount height and tilt). The fitted
plane is smoothed over frames; when a fit fails, the last good plane is held for a
few frames, after which there is no floor (None): nothing can be confirmed to stand on
it. The prior only constrains the fit; it is never used as the floor itself.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass
class FloorSettings:
    """Floor fit and classification limits (meters, degrees)."""

    floor_tolerance_m: float = 0.03      # height up to this = floor (below floor too)
    max_below_floor_m: float = 0.15      # deeper than this below the floor = bad depth
    inlier_threshold_m: float = 0.02     # RANSAC point-to-plane distance
    candidate_band_m: float = 0.15       # only points this close to the prior are tried
    fit_range_min_m: float = 0.3
    fit_range_max_m: float = 4.0
    max_tilt_change_deg: float = 10.0    # fitted normal vs prior normal
    max_height_change_m: float = 0.10    # fitted camera height vs prior
    min_inliers: int = 1000              # of ~14k sampled points; real floor gives 1k-6k
    iterations: int = 150
    sample_step: int = 4                 # use every Nth pixel in both directions
    smoothing: float = 0.5               # weight of the new fit in the running plane
    hold_frames: int = 10                # keep the last fit this many failed frames


@dataclass
class FloorPlane:
    """Plane in the camera optical frame; normal points up."""

    normal: np.ndarray
    offset: float
    source: str = 'prior'    # 'prior' (from TF), 'fit' or 'held'
    inliers: int = 0

    def height(self, points):
        """Height above the floor of optical-frame points (..., 3)."""
        return np.asarray(points) @ self.normal + self.offset

    @property
    def camera_height_m(self):
        """Camera height above the floor."""
        return self.offset

    @property
    def pitch_down_deg(self):
        """Angle the optical axis points below the floor plane."""
        return -math.degrees(math.asin(float(np.clip(self.normal[2], -1, 1))))

    @property
    def roll_deg(self):
        """Sideways tilt of the camera relative to the floor."""
        return math.degrees(math.asin(float(np.clip(self.normal[0], -1, 1))))


def prior_plane(rotation, translation):
    """Plane z = 0 of a floor frame given p_camera = rotation @ p_floor + translation."""
    normal = np.asarray(rotation, float) @ np.array([0.0, 0.0, 1.0])
    normal /= np.linalg.norm(normal)
    return FloorPlane(normal, float(-normal @ np.asarray(translation, float)), 'prior')


def depth_points(depth, matrix):
    """Back-project an optical-Z depth image (meters, NaN = none) to (H, W, 3) points."""
    height, width = depth.shape
    fx, fy, cx, cy = matrix[0, 0], matrix[1, 1], matrix[0, 2], matrix[1, 2]
    u = np.arange(width, dtype=np.float32)[None, :]
    v = np.arange(height, dtype=np.float32)[:, None]
    return np.dstack(((u - cx) * depth / fx, (v - cy) * depth / fy, depth))


def plane_from_points(points):
    """Least-squares plane through points; returns (unit normal, offset)."""
    centre = points.mean(axis=0)
    normal = np.linalg.svd(points - centre, full_matrices=False)[2][2]
    return normal, float(-normal @ centre)


def fit_floor(points, prior, settings=FloorSettings(), rng=None):
    """RANSAC floor fit near the prior; returns a FloorPlane or None."""
    rng = rng or np.random.default_rng(0)
    step = settings.sample_step
    sample = points[::step, ::step].reshape(-1, 3)
    sample = sample[np.isfinite(sample).all(axis=1)]
    distance = np.linalg.norm(sample, axis=1)
    sample = sample[(distance >= settings.fit_range_min_m)
                    & (distance <= settings.fit_range_max_m)]
    sample = sample[np.abs(prior.height(sample)) <= settings.candidate_band_m]
    if len(sample) < settings.min_inliers:
        return None
    cos_limit = math.cos(math.radians(settings.max_tilt_change_deg))
    # All candidate planes at once: 3 random points each -> normal and offset.
    picks = sample[rng.integers(0, len(sample), (settings.iterations, 3))]
    normals = np.cross(picks[:, 1] - picks[:, 0], picks[:, 2] - picks[:, 0])
    sizes = np.linalg.norm(normals, axis=1)
    usable = sizes > 1e-9
    normals = normals[usable] / sizes[usable, None]
    normals *= np.where(normals @ prior.normal < 0, -1.0, 1.0)[:, None]
    near_prior = normals @ prior.normal >= cos_limit   # drop walls and other steep planes
    normals = normals[near_prior]
    offsets = -np.einsum('ij,ij->i', normals, picks[usable][near_prior, 0])
    if not len(normals):
        return None
    # Score every candidate in one matrix product: inliers = points within threshold.
    points32 = sample.astype(np.float32)
    counts = np.concatenate([
        (np.abs(points32 @ normals[i:i + 50].T.astype(np.float32)
                + offsets[i:i + 50].astype(np.float32)) <= settings.inlier_threshold_m
         ).sum(axis=0) for i in range(0, len(normals), 50)])
    best = int(np.argmax(counts))
    best_count = int(counts[best])
    if best_count < settings.min_inliers:
        return None
    inliers = np.abs(sample @ normals[best] + offsets[best]) <= settings.inlier_threshold_m
    normal, offset = plane_from_points(sample[inliers])
    if normal @ prior.normal < 0:
        normal, offset = -normal, -offset
    if (normal @ prior.normal < cos_limit
            or abs(offset - prior.offset) > settings.max_height_change_m):
        return None
    return FloorPlane(normal, offset, 'fit', best_count)


class FloorTracker:
    """Smooth fitted planes over frames; hold the last fit briefly, then report None."""

    def __init__(self, settings=FloorSettings()):
        self.settings = settings
        self.plane = None
        self.missed = 0

    def update(self, points, prior, rng=None):
        """Fit this frame's floor; return the plane to use, or None if no floor is seen."""
        fit = fit_floor(points, prior, self.settings, rng)
        if fit is not None:
            if self.plane is not None:
                w = self.settings.smoothing
                normal = w * fit.normal + (1 - w) * self.plane.normal
                normal /= np.linalg.norm(normal)
                fit = FloorPlane(normal, w * fit.offset + (1 - w) * self.plane.offset,
                                 'fit', fit.inliers)
            self.plane, self.missed = fit, 0
        elif self.plane is not None and self.missed < self.settings.hold_frames:
            self.missed += 1
            self.plane = FloorPlane(self.plane.normal, self.plane.offset, 'held')
        else:
            self.plane = None
        return self.plane


def classify(heights, settings=FloorSettings()):
    """
    Return masks (floor, above) from a height image.

    Everything up to floor_tolerance_m is floor, including points measured below the
    plane (stereo noise near the camera and depth smearing at object bases); points more
    than max_below_floor_m below it are bad depth and, like NaN, in neither mask.
    """
    valid = np.isfinite(heights) & (heights >= -settings.max_below_floor_m)
    floor = valid & (heights <= settings.floor_tolerance_m)
    above = valid & (heights > settings.floor_tolerance_m)
    return floor, above
