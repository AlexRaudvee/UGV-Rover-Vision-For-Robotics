"""Conservative physical gaps between measured obstacle silhouettes."""

from dataclasses import asdict, dataclass
import math


@dataclass
class Gap:
    """Opening and possible center bearings after allowing for required clearance."""

    angle_min: float
    angle_max: float
    center_min: float
    center_max: float
    width_m: float
    edge_distance_m: float
    planning_distance_m: float
    clearance_m: float = 0.0

    def to_dict(self):
        """Serialize a measured gap."""
        return asdict(self)


def find_gaps(blockers, unknown_intervals=(), required_width=0.35, margin=0.0):
    """
    Find bounded openings, projected to the nearer object's optical-Z plane.

    A diagonal distance between staggered objects can overstate available lateral
    clearance. Project both facing-edge rays to the nearer plane instead. Inset
    both sides by half the required vehicle width plus optional side margin.
    Any measured obstacle can bound a gap, whatever its width; an unknown
    (unmeasured) silhouette inside an opening closes it.
    """
    if not (math.isfinite(required_width) and required_width > 0
            and math.isfinite(margin) and margin >= 0):
        raise ValueError('Gap width must be positive; side margin must be nonnegative')
    ordered = sorted(blockers, key=lambda item: item.angle_min)
    if len(ordered) < 2:
        return []
    gaps = []
    right = ordered[0]
    clearance = required_width + 2 * margin
    for left in ordered[1:]:
        if left.angle_min <= right.angle_max:
            if left.angle_max > right.angle_max:
                right = left
            continue
        low, high = right.angle_max, left.angle_min
        bounded = -math.pi/2 < low < high < math.pi/2
        unknown = any(item['angle_max'] >= low and item['angle_min'] <= high
                      for item in unknown_intervals)
        if bounded and not unknown:
            right_z = right.distance_m * math.cos(right.bearing)
            left_z = left.distance_m * math.cos(left.bearing)
            z = min(right_z, left_z)
            near_y, far_y = z * math.tan(low), z * math.tan(high)
            width = far_y - near_y
            uncertainty = 2 * (right.width_std_m + left.width_std_m)
            uncertainty += 2 * max(right.range_std_m, left.range_std_m) * (
                math.tan(high) - math.tan(low))
            lower_width = max(0, width - uncertainty)
            if z > 0 and lower_width + 1e-7 >= clearance:
                inset = (clearance + uncertainty) / 2
                center_min = math.atan((near_y + inset) / z)
                center_max = math.atan((far_y - inset) / z)
                if center_min > center_max:  # Floating point equality at exactly 35 cm.
                    center_min = center_max = (center_min + center_max) / 2
                lateral = left_z * math.tan(high) - right_z * math.tan(low)
                edge_distance = math.hypot(left_z - right_z, lateral)
                gaps.append(Gap(low, high, center_min, center_max, width, edge_distance, z,
                                lower_width))
        right = left
    return gaps
