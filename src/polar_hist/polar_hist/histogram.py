"""Aggregate obstacle angular extents, retaining unknown directions explicitly."""

import math

import numpy as np


class PolarHistogram:
    """A full-circle histogram with camera-visible bins and short temporal smoothing."""

    def __init__(self, bin_degrees=5.0, range_min=0.15, range_max=5.0,
                 smoothing_s=0.15, release_s=0.5):
        if not math.isfinite(bin_degrees) or not 0 < bin_degrees <= 90:
            raise ValueError('bin_degrees must be finite, positive, and at most 90')
        bins = round(360 / bin_degrees)
        if bins < 4 or not math.isclose(bins * bin_degrees, 360, abs_tol=1e-6):
            raise ValueError('bin_degrees must divide 360 exactly')
        if not (0 < range_min < range_max and smoothing_s >= 0 and release_s > 0):
            raise ValueError('Invalid range or smoothing settings')
        self.step = 2 * math.pi / bins
        self.angles = -math.pi + self.step * (np.arange(bins) + 0.5)
        self.range_min, self.range_max = range_min, range_max
        self.smoothing_s, self.release_s = smoothing_s, release_s
        self.scores = np.zeros(bins, dtype=np.float32)
        self.observed = np.zeros(bins, dtype=bool)
        self.ranges = np.full(bins, np.nan, dtype=np.float32)
        self.last_hit = np.full(bins, -np.inf)
        self.last_update = None

    def clear_unknown(self):
        """Invalidate immediately when images/calibration become unavailable."""
        self.scores[:] = 0
        self.observed[:] = False
        self.ranges[:] = np.nan
        self.last_hit[:] = -np.inf
        self.last_update = None

    def update(self, detections, field_of_view, now):
        """Update bin scores and camera-relative obstacle distances in meters."""
        low, high = field_of_view
        # A bin is visible if any part of its angular interval is in the camera view.
        observed = ((self.angles + self.step / 2 >= low)
                    & (self.angles - self.step / 2 <= high))
        raw = np.zeros_like(self.scores)
        current_ranges = np.full_like(self.ranges, np.inf)
        for item in detections:
            hit = ((self.angles + self.step / 2 >= item.angle_min)
                   & (self.angles - self.step / 2 <= item.angle_max) & observed)
            if not hit.any():
                continue
            distance = item.distance_m
            proximity = np.clip((self.range_max - distance)
                                / (self.range_max - self.range_min), 0, 1)
            raw[hit] = np.maximum(raw[hit], item.confidence * (0.15 + 0.85 * proximity))
            current_ranges[hit] = np.minimum(current_ranges[hit], distance)
            self.last_hit[hit] = now
        dt = 1.0 if self.last_update is None else max(0, now - self.last_update)
        alpha = 1.0 if self.smoothing_s == 0 else -math.expm1(-dt / self.smoothing_s)
        self.scores += alpha * (raw - self.scores)
        remembered = (now - self.last_hit <= self.release_s) & observed
        self.scores[~remembered] = 0
        current = np.isfinite(current_ranges)
        self.ranges[current] = current_ranges[current]
        self.ranges[observed & ~remembered] = np.inf
        self.ranges[~observed] = np.nan
        self.scores[~observed] = 0
        self.observed = observed
        self.last_update = now

    def values(self):
        """Return normalized scores, using NaN for directions not searched by the camera."""
        return np.where(self.observed, self.scores, np.nan).astype(np.float32)
