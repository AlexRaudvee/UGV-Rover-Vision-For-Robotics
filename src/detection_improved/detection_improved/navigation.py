"""Conservative navigation evidence from the existing fitted depth floor.

Only sampled, measured floor patches are free. No ray, missing pixel, empty
detection list, or semantic rejection is treated as proof of free space.
Coordinates are corrected into the nominal optical TF frame exactly once.
"""

import math

import numpy as np

from .floor import depth_points


def timing(rgb_stamp, depth_stamp, now, max_age=1., max_skew=.15):
    """Check both acquisitions, even when the display uses receipt-time freshness."""
    age = now-min(rgb_stamp, depth_stamp)
    skew = abs(rgb_stamp-depth_stamp)
    valid = all(math.isfinite(v) and v > 0 for v in (rgb_stamp, depth_stamp))
    reason = ''
    if not valid or not -.1 <= now-max(rgb_stamp, depth_stamp) or age > max_age:
        reason = 'RGB/depth acquisition is stale, future or invalid'
    elif skew > max_skew:
        reason = 'RGB/depth acquisitions are not synchronized'
    return dict(usable=not reason, reason=reason, age_s=age, pair_skew_s=skew)


def evidence(depth, matrix, plane, correction, support=None, blockers=(), unknown=(),
             step=4, max_points=4000, minimum=.15, maximum=5.,
             floor_tolerance=.025, min_height=.04, max_height=.45):
    """Sparse free patches, height-filtered hits and ALL measured blocker faces."""
    result = dict(free_points=[], occupied_points=[], blocker_faces=[],
                  sampled_pixels=0, supported_pixels=0, unknown_pixels=0)
    if plane is None or plane.source not in ('fit', 'held'):
        return dict(result, usable=False, reason='No fitted floor plane')
    matrix = np.array(matrix, float)
    matrix[:2] /= step
    sampled = depth[::step, ::step]
    points = depth_points(sampled, matrix)
    heights = plane.height(points)
    valid = np.isfinite(points).all(axis=2) & (sampled >= minimum) & (sampled <= maximum)
    if support is not None:
        if support.shape != depth.shape:
            raise ValueError('Measured-depth support and depth dimensions differ')
        valid &= support[::step, ::step].astype(bool)
    floor = valid & (np.abs(heights) <= floor_tolerance)
    occupied = valid & (heights >= min_height) & (heights <= max_height)
    bearings = np.arctan2(-points[:, :, 0], points[:, :, 2])
    for interval in unknown:
        floor &= ~((bearings >= interval['angle_min']) & (bearings <= interval['angle_max']))

    def limited(mask):
        selected = points[mask]
        stride = max(1, math.ceil(len(selected)/max_points))
        # correction maps nominal -> actual optical coordinates; undo it for TF.
        return np.round(selected[::stride] @ correction, 4).tolist()

    result.update(free_points=limited(floor), occupied_points=limited(occupied),
                  sampled_pixels=int(valid.size), supported_pixels=int(valid.sum()),
                  unknown_pixels=int((~valid).sum()))
    # Width-band membership is informational. Even an out-of-band or clipped
    # measured obstacle contributes a front face. Its rear extent is unknown.
    if abs(plane.normal[1]) > .1:
        for item in blockers:
            z = item.distance_m*math.cos(item.bearing)
            x = -z*np.tan([item.angle_min, item.angle_max])
            y = (-plane.offset-plane.normal[0]*x-plane.normal[2]*z)/plane.normal[1]
            face = np.column_stack((x, y, np.full(2, z))) @ correction
            if np.isfinite(face).all():
                result['blocker_faces'].append(dict(points=np.round(face, 4).tolist(),
                    range_std_m=float(item.range_std_m), width_std_m=float(item.width_std_m),
                    target_size=bool(item.accepted), clipped=bool(item.clipped)))
    return dict(result, usable=True, reason='')
