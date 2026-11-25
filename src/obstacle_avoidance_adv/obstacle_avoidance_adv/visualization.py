"""Read-only motion intent for Vizanti; never changes a controller or its commands."""
import math

import numpy as np

from .planner import BACKOFF_M, ESCAPE_M, wrap


def motion_intent(controller, pose, command, waiting_reason=''):
    """Describe the sent velocity and the current leg, in continuous motion coordinates."""
    pose = np.asarray(pose, dtype=float)
    v, w = command
    result = dict(kind='stop', label=waiting_reason or controller.reason,
                  path=[], remaining_m=None, remaining_deg=None)
    if not controller.active or abs(v) + abs(w) < 1e-6:
        return result

    state = controller.state
    heading, remaining = None, None
    if state in ('APPROACH', 'REJOIN'):
        delta = controller.goal - pose[:2]
        heading, remaining = math.atan2(delta[1], delta[0]), float(np.linalg.norm(delta))
    elif state in ('TURN_OUT', 'TURN_FORWARD', 'GAP_ALIGN', 'LOOK'):
        heading = controller.turn_heading
    elif state == 'SIDESTEP':
        heading = controller.turn_heading
        remaining = controller.side_length - float((pose[:2] - controller.leg_start) @
                    np.array([math.cos(heading), math.sin(heading)]))
    elif state == 'PASS':
        heading = controller.leg_heading
        remaining = controller.cfg.pass_distance - float((pose[:2] - controller.leg_start) @
                    np.array([math.cos(heading), math.sin(heading)]))
    elif state == 'GAP':
        delta = controller.gap_entry - pose[:2]
        heading = math.atan2(delta[1], delta[0]) if np.linalg.norm(delta) > .10 else controller.gap_heading
        remaining = float(delta @ np.array([math.cos(controller.gap_heading), math.sin(controller.gap_heading)]))
    elif state == 'GAP_THROUGH':
        remaining = controller.gap_through - float((pose[:2] - controller.leg_start) @
                    np.array([math.cos(controller.leg_heading), math.sin(controller.leg_heading)]))
    elif state == 'ESCAPE':
        heading = controller.escape_heading
        remaining = ESCAPE_M - float((pose[:2] - controller.leg_start) @
                    np.array([math.cos(heading), math.sin(heading)]))

    if abs(v) < 1e-6:
        angle = wrap(heading - pose[2]) if heading is not None else w * 2.
        if angle * w <= 0:
            angle = w * 2.
        result.update(kind='turn', remaining_deg=abs(math.degrees(angle)),
                      label=f'TURN {"LEFT" if w > 0 else "RIGHT"} · {abs(math.degrees(angle)):.0f}° remaining')
        # Draw a direction arrow around the rover, not a translation toward a stale waypoint.
        result['path'] = [(pose[:2] + .28 * np.array([math.cos(pose[2] + angle*t),
                           math.sin(pose[2] + angle*t)])).tolist() for t in np.linspace(0., 1., 21)]
        return result

    if v < 0:
        remaining = (max(0., BACKOFF_M - float(np.linalg.norm(pose[:2] - controller.backoff_start)))
                     if controller.backoff_start is not None else None)
    remaining = max(0., remaining) if remaining is not None else None
    result.update(kind='reverse' if v < 0 else 'drive', remaining_m=remaining,
                  label=('REVERSE' if v < 0 else state) +
                        (f' · {remaining:.2f} m remaining' if remaining is not None else ''))
    # Preview at most two seconds / 0.6 m of the actual command, including curved motion.
    distance = min(.6, abs(v) * 2., remaining if remaining is not None else .6)
    for t in np.linspace(0., distance / abs(v), 13):
        if abs(w) < 1e-6:
            point = pose[:2] + v*t*np.array([math.cos(pose[2]), math.sin(pose[2])])
        else:
            point = pose[:2] + (v/w)*np.array([math.sin(pose[2]+w*t)-math.sin(pose[2]),
                                             math.cos(pose[2])-math.cos(pose[2]+w*t)])
        result['path'].append(point.tolist())
    return result
