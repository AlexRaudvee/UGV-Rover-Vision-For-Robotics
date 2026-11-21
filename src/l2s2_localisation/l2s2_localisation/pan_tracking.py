"""Frame-count hysteresis and bounded pan motion, independent of ROS."""
import math


class PanController:
    def __init__(self, ids, missing_frames=3, timeout=.8, speed=8., gain=.7,
                 deadband=3., alpha=.25, limit=120., one_marker='center',
                 search_reach=36.):
        # What to do when only ONE tag is in view:
        #   'center': turn back to 0 deg (original behaviour)
        #   'search': keep the tag in the image and swing the camera up to
        #             search_reach deg to either side of it until a second tag appears
        if one_marker not in ('center', 'search'):
            raise ValueError("one_marker must be 'center' or 'search'")
        if not math.isfinite(search_reach) or not 0 <= search_reach < 90:
            raise ValueError('search_reach must be between 0 and 90 deg')
        self.one_marker, self.search_reach, self.search_dir = one_marker, search_reach, 1.
        if len(ids) < 2 or len(set(ids)) != len(ids) or any(type(tag) is not int or tag < 0 for tag in ids):
            raise ValueError('Specify at least two distinct nonnegative integer target IDs')
        if type(missing_frames) is not int or missing_frames < 1:
            raise ValueError('missing_frames must be a positive integer')
        if not all(math.isfinite(v) for v in (timeout, speed, gain, deadband, alpha, limit)):
            raise ValueError('Controller settings must be finite')
        if timeout <= 0 or speed <= 0 or gain <= 0 or deadband < 0 or not 0 < alpha <= 1 or not 0 < limit <= 180:
            raise ValueError('Invalid controller settings')
        self.ids, self.grace, self.timeout = ids, missing_frames, timeout
        self.speed, self.gain, self.deadband, self.alpha, self.limit = speed, gain, deadband, alpha, limit
        self.pan = 0.
        self.tracks = {}
        self.last_frame = None
        self.mode = 'centering'
        self.tracked_ids = []

    def observe(self, bearings, now, exposure_pan=None):
        """Bearings are relative to the optical axis; retain robot-relative angles."""
        self.last_frame = now
        for tag in self.ids:
            previous = self.tracks.get(tag)
            if tag in bearings:
                angle = (self.pan if exposure_pan is None else exposure_pan) + bearings[tag]
                expired = previous is None or previous['misses'] >= self.grace or now-previous['time'] > self.timeout
                smooth = angle if expired else previous['angle'] + self.alpha*(angle-previous['angle'])
                self.tracks[tag] = {'angle': smooth, 'misses': 0, 'time': now}
            elif previous is not None:
                previous['misses'] += 1

    def step(self, dt, now):
        if not math.isfinite(dt) or dt < 0:
            raise ValueError('dt must be nonnegative and finite')
        self.tracked_ids = [tag for tag in self.ids if tag in self.tracks and
                            self.tracks[tag]['misses'] < self.grace and
                            now-self.tracks[tag]['time'] <= self.timeout]
        if len(self.tracked_ids) >= 2:
            angles = [self.tracks[tag]['angle'] for tag in self.tracked_ids]
            # Center the outermost bearings, so a cluster on one side does not
            # pull the camera away from another visible marker at the far edge.
            target = (min(angles)+max(angles))/2
            error = target-self.pan
            target = self.pan if abs(error) <= self.deadband else self.pan+self.gain*error
            self.mode = 'tracking_markers'
        elif len(self.tracked_ids) == 1 and self.one_marker == 'search':
            angle = self.tracks[self.tracked_ids[0]]['angle']
            target = max(-self.limit, min(self.limit, angle + self.search_dir*self.search_reach))
            if abs(target-self.pan) <= 1.:
                # Reached this side (tag now at the image edge): look the other way.
                self.search_dir = -self.search_dir
            self.mode = 'searching_with_one_marker'
        else:
            target, self.mode = 0., 'centering'
        target = max(-self.limit, min(self.limit, target))
        # A delayed timer must not produce a large physical jump.
        maximum = self.speed * min(dt, .1)
        self.pan += max(-maximum, min(maximum, target-self.pan))
        return self.pan
