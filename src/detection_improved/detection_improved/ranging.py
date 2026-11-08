"""Short range histories matched by image overlap, with no missing-data fallback."""

from collections import deque
from dataclasses import dataclass, field, replace
import math

import numpy as np

from .perception import RangeEstimate


def overlap(first, second):
    """Return bounding box intersection over union."""
    ax, ay, aw, ah = first
    bx, by, bw, bh = second
    intersection = max(0, min(ax + aw, bx + bw) - max(ax, bx)) * max(
        0, min(ay + ah, by + bh) - max(ay, by))
    return intersection / max(1, aw * ah + bw * bh - intersection)


@dataclass
class Track:
    """A silhouette's recent supported measurements and pending discontinuity."""

    bbox: tuple
    seen: float
    history: deque = field(default_factory=lambda: deque(maxlen=9))
    pending: object = None
    ema_m: float = math.nan
    ema_stamp: float = math.nan


class TemporalRanges:
    """Smooth only current supported measurements; a missing sample stays unknown."""

    def __init__(self, use_ema=False):
        self.use_ema = use_ema
        self.tracks = []
        self.last_time = -math.inf

    def clear(self):
        """Forget history on a camera frame or clock change."""
        self.tracks = []
        self.last_time = -math.inf

    def update(self, blobs, estimates, now, settings):
        """Match silhouettes, reject jumps, and apply the selected temporal filter."""
        if now < self.last_time:
            self.clear()
        self.last_time = now
        self.tracks = [track for track in self.tracks
                       if now - track.seen <= settings.range_track_timeout_s]
        available = list(self.tracks)
        result = []
        for blob, estimate in zip(blobs, estimates):
            track = max(available, key=lambda item: overlap(item.bbox, blob.bbox), default=None)
            if track is None or overlap(track.bbox, blob.bbox) < 0.25:
                track = Track(blob.bbox, now)
                self.tracks.append(track)
            else:
                available.remove(track)
            track.bbox, track.seen = blob.bbox, now
            while track.history and now - track.history[0][0] > settings.range_track_timeout_s:
                track.history.popleft()
            if not math.isfinite(estimate.axial_m) or estimate.axial_m <= 0:
                result.append(RangeEstimate())
                track.pending = None
                continue
            if track.history:
                previous = float(np.median([item.axial_m for _, item in track.history]))
                gate = max(settings.range_jump_m, settings.range_jump_fraction * previous,
                           3 * estimate.std_m)
                if abs(estimate.axial_m - previous) > gate:
                    if (track.pending is None
                            or abs(estimate.axial_m - track.pending.axial_m) > gate / 2):
                        track.pending = estimate
                        result.append(RangeEstimate())
                        continue
                    track.history.clear()  # Confirmed new surface; do not blend old and new.
                    track.ema_m = track.ema_stamp = math.nan
            track.pending = None
            track.history.append((now, estimate))
            history = [item.axial_m for _, item in list(track.history)[-settings.range_window:]]
            median = float(np.median(history))
            spread = 1.4826 * float(np.median(np.abs(np.asarray(history) - median)))
            if self.use_ema:
                dt = now - track.ema_stamp
                tau = settings.depth_ema_tau_s
                reset = (not math.isfinite(track.ema_m) or not math.isfinite(dt)
                         or dt <= 0 or dt > settings.range_track_timeout_s or tau == 0)
                alpha = 1.0 if reset else -math.expm1(-dt / tau)
                filtered = (estimate.axial_m if reset else
                            track.ema_m + alpha * (estimate.axial_m - track.ema_m))
                track.ema_m, track.ema_stamp = filtered, now
                # Preserve sensor spread and allow for lag; smoothing must not make
                # a moving or uncertain obstacle appear more precisely measured.
                uncertainty = max(estimate.std_m, spread, abs(estimate.axial_m - filtered))
                result.append(replace(estimate, axial_m=filtered, std_m=uncertainty,
                                      raw_axial_m=estimate.axial_m,
                                      filter_method='ema' if tau > 0 else 'none', ema_alpha=alpha))
            else:
                result.append(replace(estimate, axial_m=median,
                                      std_m=max(estimate.std_m, spread),
                                      raw_axial_m=estimate.axial_m, filter_method='median'))
        return result
