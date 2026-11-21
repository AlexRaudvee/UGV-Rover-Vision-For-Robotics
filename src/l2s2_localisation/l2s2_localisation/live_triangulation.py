"""Live range-only position solving; bearings select branches and heading.

No odometry, previous position, or starting-pose prior is accepted here.
The scanner's circle/linear solvers supply seeds; damped range-only least
squares handles noisy circles, with explicit ambiguity and geometry gates.
"""
import math
from itertools import combinations

import numpy as np

from .triangulation import (Field, circle_intersections, heading_and_bearing_error,
                            linear_least_squares, rms_range_error)


DEFAULTS = dict(max_image_age_s=1., stale_after_s=1., min_baseline_m=.3,
                max_range_rms_m=.5, max_bearing_rms_deg=12.,
                ambiguity_range_margin_m=.05, ambiguity_bearing_margin_deg=3.,
                min_geometry_ratio=.02, range_sigma_m=.15)


def validate_config(settings):
    config = {**DEFAULTS, **settings}
    for key in DEFAULTS:
        value = config[key]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} must be a positive finite number')
    if config['min_geometry_ratio'] >= 1:
        raise ValueError('min_geometry_ratio must be less than 1')
    return config


def range_refine(seed, markers):
    position = np.asarray(seed, dtype=float).copy()
    points = np.asarray([m.position for m in markers], dtype=float)
    ranges = np.asarray([m.range for m in markers])
    for _ in range(40):
        offsets = position-points
        distances = np.maximum(np.linalg.norm(offsets, axis=1), 1e-9)
        step = np.linalg.lstsq(offsets/distances[:, None], ranges-distances, rcond=None)[0]
        old = float(np.sum((distances-ranges)**2))
        scale = 1.
        while scale >= 1e-5:
            trial = position+scale*step
            cost = float(np.sum((np.linalg.norm(trial-points, axis=1)-ranges)**2))
            if cost <= old+1e-12:
                position = trial
                break
            scale /= 2
        if scale < 1e-5 or np.linalg.norm(scale*step) < 1e-8:
            break
    return position


def solve(markers, field_model=Field(), settings=None):
    """Compute a fix or raise ValueError when a defensible fix is unavailable."""
    config = validate_config(settings or {})
    markers = list(markers)
    if len(markers) < 2:
        raise ValueError('need at least two valid markers from the same image')
    if len({m.label for m in markers}) != len(markers):
        raise ValueError('duplicate marker IDs in one image')
    for marker in markers:
        if len(marker.position) != 2 or not all(math.isfinite(v) for v in marker.position) or not math.isfinite(marker.range) or marker.range <= 0:
            raise ValueError('invalid marker position or range')
        if marker.bearing is not None and not math.isfinite(marker.bearing):
            raise ValueError('invalid marker bearing')
    pairs = [(a, b) for a, b in combinations(markers, 2)
             if math.dist(a.position, b.position) >= config['min_baseline_m']]
    if not pairs:
        raise ValueError('marker baseline is too short')
    seeds, approximate = [], False
    if len(markers) >= 3:
        linear = linear_least_squares(markers)
        if linear is not None:
            seeds.append(linear)
    for a, b in pairs:
        points, meet = circle_intersections(a.position, a.range, b.position, b.range)
        seeds.extend(points)
        approximate |= not meet
    candidates = []
    for seed in seeds:
        point = range_refine(seed, markers)
        if np.all(np.isfinite(point)) and not any(np.linalg.norm(point-c) < .01 for c in candidates):
            candidates.append(point)
    inside = [p for p in candidates if field_model.contains(p)]
    if not inside:
        raise ValueError('all range solutions lie outside the configured field')
    scored = sorted(((rms_range_error(p, markers), i, p) for i, p in enumerate(inside)), key=lambda s: s[0])
    best_rms = scored[0][0]
    if best_rms > config['max_range_rms_m']:
        raise ValueError('marker distances are inconsistent (range residual too large)')
    equivalent = [s for s in scored if s[0] <= min(config['max_range_rms_m'], best_rms+config['ambiguity_range_margin_m'])]
    reason = 'field bounds' if len(inside) < len(candidates) and len(equivalent) == 1 else 'range least squares'
    if len(equivalent) > 1:
        if sum(m.bearing is not None for m in markers) < 2:
            raise ValueError('mirror ambiguity: at least two bearings are needed')
        bearings = sorted([(heading_and_bearing_error(s[2], markers)[1], s[1], s) for s in equivalent], key=lambda s: s[0])
        if bearings[1][0]-bearings[0][0] < math.radians(config['ambiguity_bearing_margin_deg']):
            raise ValueError('mirror ambiguity: bearings do not distinguish the solutions reliably')
        selected = bearings[0][2]
        reason = 'camera bearings reject the mirror solution'
    else:
        selected = equivalent[0]
    residual, _, position = selected
    heading, bearing_error = heading_and_bearing_error(position, markers)
    if bearing_error is not None and bearing_error > math.radians(config['max_bearing_rms_deg']):
        raise ValueError('marker bearings are inconsistent with the range solution')
    offsets = position-np.asarray([m.position for m in markers])
    distances = np.maximum(np.linalg.norm(offsets, axis=1), 1e-9)
    jacobian = offsets/distances[:, None]
    singular = np.linalg.svd(jacobian, compute_uv=False)
    ratio = float(singular[-1]/singular[0])
    if ratio < config['min_geometry_ratio']:
        raise ValueError('marker geometry poorly constrains position')
    variance = max(config['range_sigma_m']**2, residual**2)
    covariance = variance*np.linalg.inv(jacobian.T@jacobian)
    return dict(position_m=[float(position[0]), float(position[1]), 0.],
                heading_deg=None if heading is None else math.degrees(heading),
                covariance_xy=covariance.tolist(), rms_range_error_m=residual,
                rms_bearing_error_deg=None if bearing_error is None else math.degrees(bearing_error),
                geometry_ratio=ratio, used_ids=[m.label for m in markers],
                candidates_m=[p.tolist() for p in candidates], ambiguity_resolution=reason,
                method='two range circles' if len(markers) == 2 else f'range least squares ({len(markers)} markers)',
                warnings=['noisy range circles do not all intersect exactly'] if approximate else [])
