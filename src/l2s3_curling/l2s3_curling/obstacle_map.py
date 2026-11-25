"""Preview occupancy from improved depth evidence and configured stand footprints.

Live evidence is stored in continuous odom, then projected with the current
map <- odom correction. Expiry makes space UNKNOWN, never free. No motor code.
"""

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class GridSettings:
    resolution: float = .05
    half_length: float = 4.5
    half_width: float = 3.
    obstacle_memory_s: float = 3.
    free_memory_s: float = 1.
    half_robot_length: float = .125
    half_robot_width: float = .11
    clearance: float = .06

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError('Grid dimensions, memory and footprint must be positive finite numbers')
        if self.half_length*self.half_width*4/self.resolution**2 > 1_000_000:
            raise ValueError('Obstacle grid is too large')

    @property
    def radius(self):
        return math.hypot(self.half_robot_length, self.half_robot_width)+self.clearance


@dataclass(frozen=True)
class Stand:
    label: str
    x: float
    y: float
    width: float = .30
    depth: float = .30
    yaw: float = 0.
    measured: bool = False

    def __post_init__(self):
        if (not all(math.isfinite(v) for v in (self.x, self.y, self.width, self.depth, self.yaw))
                or self.width <= 0 or self.depth <= 0):
            raise ValueError('Invalid stand footprint')


def transform_xy(points, transform):
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    x, y, h = transform
    c, s = math.cos(h), math.sin(h)
    return points @ np.array([[c, s], [-s, c]]) + [x, y]


class ObstacleMap:
    def __init__(self, settings=GridSettings(), stands=()):
        self.settings, self.stands = settings, list(stands)
        self.width = math.ceil(2*settings.half_length/settings.resolution)
        self.height = math.ceil(2*settings.half_width/settings.resolution)
        self.origin = (-settings.half_length, -settings.half_width)
        self.cells = {}  # odom voxel -> (occupied, acquisition time, uncertainty)
        self.face_tracks = []
        yy, xx = np.indices((self.height, self.width))
        self.xy = np.stack((self.origin[0]+(xx+.5)*settings.resolution,
                            self.origin[1]+(yy+.5)*settings.resolution), axis=-1)

    def clear(self):
        self.cells.clear()
        self.face_tracks.clear()

    def update(self, free, occupied, stamp, uncertainty=.025):
        if not math.isfinite(stamp) or not math.isfinite(uncertainty) or uncertainty < 0:
            raise ValueError('Invalid evidence time or uncertainty')
        for kind, points in ((False, free), (True, occupied)):
            points = np.asarray(points, dtype=float).reshape(-1, 2)
            if not np.isfinite(points).all():
                raise ValueError('Nonfinite obstacle map evidence')
            keys = np.unique(np.floor(points/self.settings.resolution).astype(np.int64), axis=0)
            for key in map(tuple, keys):
                old = self.cells.get(key)
                if old is None or stamp >= old[1]:
                    padding = uncertainty if kind else 0.
                    if old is not None and old[0] and kind and stamp == old[1]:
                        padding = max(padding, old[2])
                    self.cells[key] = (kind, stamp, padding)

    def add_face(self, endpoints, stamp, uncertainty=0.):
        endpoints = np.asarray(endpoints, float).reshape(2, 2)
        n = max(2, math.ceil(np.linalg.norm(endpoints[1]-endpoints[0])/self.settings.resolution)+1)
        self.update([], np.linspace(endpoints[0], endpoints[1], n), stamp, uncertainty)

    def add_square(self, endpoints, camera, stamp, size=.30, uncertainty=0.):
        """Place an assumed square behind the measured front, viewed from camera."""
        endpoints = np.asarray(endpoints, float).reshape(2, 2)
        camera = np.asarray(camera, float).reshape(2)
        tangent = endpoints[1] - endpoints[0]
        length = np.linalg.norm(tangent)
        if not np.isfinite(endpoints).all() or not np.isfinite(camera).all() or length < 1e-6:
            raise ValueError('Invalid detected front for square footprint')
        if not math.isfinite(size) or not 0 < size <= 2.:
            raise ValueError('Invalid assumed obstacle size')
        tangent /= length
        normal = np.array([-tangent[1], tangent[0]])
        centre = endpoints.mean(axis=0)
        if np.dot(normal, centre-camera) < 0:
            normal = -normal
        n = max(2, math.ceil(size/self.settings.resolution)*2+1)
        u, v = np.meshgrid(np.linspace(-size/2, size/2, n), np.linspace(0, size, n))
        points = centre + u[..., None]*tangent + v[..., None]*normal
        self.update([], points.reshape(-1, 2), stamp, uncertainty)

    def update_faces(self, faces, camera, stamp):
        """Confirm a face in four distinct frames within two capture seconds."""
        tracks = []
        for track in self.face_tracks:
            recent = [sample for sample in track if stamp - sample[0] <= 2.]
            if recent:
                tracks.append(recent)
        unmatched = set(range(len(tracks)))
        for endpoints, uncertainty in faces:
            if np.linalg.norm(endpoints[1] - endpoints[0]) < 1e-6:
                continue
            centre = endpoints.mean(axis=0)
            distance = lambda index: np.linalg.norm(centre - tracks[index][-1][1].mean(axis=0))
            index = min(unmatched, key=distance, default=None)
            sample = (stamp, endpoints, camera, uncertainty)
            if index is None or distance(index) > .25:
                tracks.append([sample])
            else:
                unmatched.remove(index)
                tracks[index] = (tracks[index] + [sample])[-5:]
        self.face_tracks = tracks
        # Replace the previous footprints so jitter does not leave a trail.
        self.cells = {key: value for key, value in self.cells.items() if not value[0]}
        for track in tracks:
            if len(track) < 4:
                continue
            latest_stamp, reference, camera, _ = track[-1]
            tangent = reference[1] - reference[0]
            aligned = [edge if np.dot(edge[1] - edge[0], tangent) >= 0 else edge[::-1]
                       for _, edge, _, _ in track]
            average = np.mean(aligned, axis=0)
            self.add_square(average, camera, latest_stamp,
                            uncertainty=max(sample[3] for sample in track))

    def render(self, transform, now, allow_free=True, include_static=True):
        grid = np.full((self.height, self.width), -1, dtype=np.int8)
        uncertainty = np.zeros(grid.shape, dtype=np.float32)
        # Global footprints remain fixed when the EKF corrects live observations.
        for stand in self.stands if include_static else ():
            delta = self.xy-[stand.x, stand.y]
            c, s = math.cos(stand.yaw), math.sin(stand.yaw)
            u, v = delta[..., 0]*c+delta[..., 1]*s, -delta[..., 0]*s+delta[..., 1]*c
            hit = (abs(u) <= stand.depth/2) & (abs(v) <= stand.width/2)
            grid[hit] = 100
        # The configured playing-field edge is a navigation boundary.
        if include_static:
            grid[[0, -1], :] = 100
            grid[:, [0, -1]] = 100
        for key, record in list(self.cells.items()):
            occupied, stamp, padding = record
            ttl = self.settings.obstacle_memory_s if occupied else self.settings.free_memory_s
            if not -.1 <= now-stamp <= ttl:
                del self.cells[key]
        if transform is not None:
            for occupied in (False, True):
                records = [(key, rec) for key, rec in self.cells.items() if rec[0] == occupied]
                if not records or (not occupied and not allow_free):
                    continue
                points = (np.array([key for key, _ in records])+.5)*self.settings.resolution
                mapped = transform_xy(points, transform)
                indices = np.floor((mapped-self.origin)/self.settings.resolution).astype(int)
                for (ix, iy), (_, record) in zip(indices, records):
                    if 0 <= ix < self.width and 0 <= iy < self.height:
                        if occupied:
                            grid[iy, ix] = 100
                            uncertainty[iy, ix] = max(uncertainty[iy, ix], record[2])
                        elif grid[iy, ix] != 100:
                            grid[iy, ix] = 0
        inflated = grid.copy()
        occupied = grid == 100
        # Include voxel size and reported obstacle uncertainty conservatively.
        radius = self.settings.radius + self.settings.resolution/math.sqrt(2)
        if occupied.any():
            extra = float(uncertainty.max())
            distance = cv2.distanceTransform((~occupied).astype(np.uint8), cv2.DIST_L2, 5)
            inflated[distance*self.settings.resolution <= radius+extra] = 100
        return grid, inflated
