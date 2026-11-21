"""Straight-line path from the robot's estimated position to a field target (no ROS).

Field frame: origin at the centre spot, x towards the goal with the targets, y left.
The line is planned once and kept; it is planned again only when
  * the position estimate jumps (a marker corrected it), or
  * the robot is too far off the line, or
  * the target changes.
"""
from dataclasses import dataclass
import math

# Session 3 targets, assignment Table 2 (metres, as given there).
TARGETS = {
    1: ('Penalty mark', 3.20, 0.00),
    2: ('Goal area corner right', 3.90, 1.10),
    3: ('Goal area corner left', 3.90, -1.10),
    4: ('Penalty area corner right', 2.85, 2.00),
    5: ('Penalty area corner left', 2.85, -2.00),
}


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Plan:
    start: tuple          # (x, y) where the line starts
    goal: tuple           # (x, y) target
    reason: str           # why this plan was made
    number: int           # 1 for the first plan, 2 after the first re-plan, ...


def line_points(start, goal, spacing=0.1):
    """Points from start to goal, about `spacing` metres apart (both ends included)."""
    length = math.dist(start, goal)
    steps = max(1, math.ceil(length / spacing))
    return [(start[0] + (goal[0] - start[0]) * k / steps,
             start[1] + (goal[1] - start[1]) * k / steps) for k in range(steps + 1)]


def along_and_offset(position, start, goal):
    """Distance travelled along the line, and sideways offset from it (left positive)."""
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return 0.0, 0.0
    ux, uy = dx / length, dy / length
    vx, vy = position[0] - start[0], position[1] - start[1]
    return vx * ux + vy * uy, ux * vy - uy * vx


class PathPlanner:

    def __init__(self, goal, max_offset=0.3, max_jump=0.3, arrive=0.15):
        self.goal = tuple(goal)
        self.max_offset, self.max_jump, self.arrive = max_offset, max_jump, arrive
        self.plan = None
        self.last = None
        self.count = 0

    def set_goal(self, goal):
        self.goal = tuple(goal)
        self.plan = None              # the next update plans a new line

    def update(self, pose):
        """Feed the latest pose (x, y, heading). Returns (status dict, replanned?)."""
        x, y, heading = pose
        position = (x, y)
        reason = None
        if self.plan is None:
            reason = 'new target' if self.count else 'first plan'
        elif math.dist(position, self.goal) > self.arrive:
            jump = math.dist(position, self.last) if self.last else 0.0
            _, offset = along_and_offset(position, self.plan.start, self.plan.goal)
            if jump > self.max_jump:
                reason = f'position jumped {jump:.2f} m'
            elif abs(offset) > self.max_offset:
                reason = f'{abs(offset):.2f} m off the line'
        if reason:
            self.count += 1
            self.plan = Plan(position, self.goal, reason, self.count)
        self.last = position
        return self.status(pose), reason is not None

    def status(self, pose):
        x, y, heading = pose
        along, offset = along_and_offset((x, y), self.plan.start, self.plan.goal)
        distance = math.dist((x, y), self.goal)
        to_goal = math.atan2(self.goal[1] - y, self.goal[0] - x)
        return {'target': list(self.goal), 'distance_m': round(distance, 3),
                'offset_m': round(offset, 3),
                'progress_m': round(along, 3),
                'line_length_m': round(math.dist(self.plan.start, self.plan.goal), 3),
                'heading_error_deg': round(math.degrees(wrap(to_goal - heading)), 1),
                'arrived': distance <= self.arrive,
                'plan_number': self.plan.number, 'plan_reason': self.plan.reason,
                'plan_start': [round(v, 3) for v in self.plan.start]}
