"""Eight-second, map-guided look-and-hold camera search."""
import math


class RelocalizationSearch:
    def __init__(self, start_stamp, family, limit, last_bearing=None):
        self.start_stamp = start_stamp
        self.family = family
        self.limit = limit
        self.held = set()
        self.skipped = set()
        self.hold_until = None
        self.holding_ids = set()
        self.target_id = None
        self.arrived_since = None
        self.sweep_direction = 1
        self.last_bearing = last_bearing

    def step(self, control, now, pose, markers, accepted):
        visible = set(control.tracked_ids)
        new_visible = visible - self.held
        if new_visible and self.hold_until is None:
            self.holding_ids = set(visible)
            self.hold_until = now + 1.
            self.target_id = None
        if self.hold_until is not None:
            if now < self.hold_until and visible:
                angles = [control.tracks[tag]['angle'] for tag in visible]
                return (min(angles)+max(angles))/2, 'relocalize_hold'
            self.held.update(self.holding_ids)
            self.hold_until = None

        if not visible and self.last_bearing is not None:
            target = max(-self.limit, min(self.limit, self.last_bearing))
            if abs(control.pan-target) > 2.:
                return target, 'relocalize_last_tag'
            self.last_bearing = None
        elif visible:
            self.last_bearing = None

        candidates = {}
        if pose is not None:
            x, y, heading = pose
            for tag, position in markers.items():
                if tag not in control.ids or tag in self.skipped:
                    continue
                label = f'{self.family}:{tag}'
                if accepted.get(label, 0.) > self.start_stamp or tag in self.held:
                    continue
                angle = math.degrees(math.atan2(position[1]-y, position[0]-x)-heading)
                angle = (angle+180.) % 360. - 180.
                if abs(angle) <= self.limit:
                    candidates[tag] = angle
        if self.target_id not in candidates:
            self.target_id = min(candidates, key=lambda tag: abs(candidates[tag]-control.pan)) if candidates else None
            self.arrived_since = None
        if self.target_id is not None:
            target = candidates[self.target_id]
            if abs(control.pan-target) <= 2. and self.arrived_since is None:
                self.arrived_since = now
            if self.arrived_since is not None and now-self.arrived_since >= 1.:
                self.skipped.add(self.target_id)
                self.target_id = None
            else:
                return target, 'relocalize_search'
        if abs(control.pan-self.sweep_direction*self.limit) <= 1e-9:
            self.sweep_direction *= -1
        return self.sweep_direction*self.limit, 'relocalize_sweep'
