"""The avoidance state machine. It has no ROS code, so it can be tested on a laptop.

The robot's start pose defines the "path": x points along the start heading and
y = 0 is the path itself. The states are:

  CRUISE  follow the path until an obstacle is close
  TURN    turn on the spot to a chosen heading
  PASS    drive to a lane beside the obstacle and follow it until past the obstacle
  RETURN  drive back to the path
  HOLD    stopped, because no safe heading exists right now
  DONE    finished or gave up
"""

from dataclasses import dataclass, field
import math


CRUISE, TURN, PASS, RETURN, HOLD, DONE = 'CRUISE', 'TURN', 'PASS', 'RETURN', 'HOLD', 'DONE'


@dataclass
class Settings:
    """All distances are in meters, speeds in m/s and rad/s, and times in seconds."""

    cruise_speed: float = 0.15
    pass_speed: float = 0.12
    creep_speed: float = 0.08
    max_turn_rate: float = 0.5
    heading_gain: float = 1.5
    heading_tolerance: float = 0.08
    target_distance: float = 2.0
    robot_width: float = 0.35
    side_margin: float = 0.08
    trigger_distance: float = 1.0
    plan_range: float = 2.5
    camera_forward: float = 0.07
    pass_length: float = 0.65
    line_lookahead: float = 0.8
    rejoin_lookahead: float = 0.6
    line_tolerance: float = 0.05
    max_lateral: float = 1.0
    max_heading: float = 1.2
    stop_distance: float = 0.35
    body_front: float = 0.15
    stop_half_width: float = 0.20
    min_stop_points: int = 2
    data_timeout: float = 0.8
    blocked_timeout: float = 15.0
    max_run_time: float = 120.0
    preferred_side: str = 'auto'

    @property
    def clearance(self):
        """Distance to keep between the robot's center line and an obstacle."""
        return self.robot_width / 2 + self.side_margin


@dataclass
class Obstacle:
    """A measured object, using the detector's angles (positive is left)."""

    bearing: float
    angle_min: float
    angle_max: float
    distance: float


@dataclass
class GapOption:
    """A gap from polar_hist: the directions to aim for and the gap's distance."""

    center_min: float
    center_max: float
    distance_z: float


@dataclass
class Perception:
    """What the robot sees at one moment."""

    received: float
    obstacles: list = field(default_factory=list)
    unknown: list = field(default_factory=list)
    gaps: list = field(default_factory=list)


@dataclass
class Plan:
    """The decision that stays fixed for one obstacle."""

    heading: float
    lane_y: float
    clear_x: float
    side: str


@dataclass
class Command:
    """The output of one control cycle."""

    linear: float
    angular: float
    state: str
    note: str = ''


def wrap(angle):
    """Wrap an angle to the range -pi to pi."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    """Limit a value to a range."""
    return max(low, min(high, value))


def edges(obstacle, settings):
    """Return (forward, right edge y, left edge y) relative to the robot's center."""
    z = obstacle.distance * math.cos(obstacle.bearing)
    return (z + settings.camera_forward,
            z * math.tan(obstacle.angle_min),
            z * math.tan(obstacle.angle_max))


def in_path(obstacle, settings):
    """Check if an obstacle overlaps the strip straight ahead of the robot."""
    forward, right, left = edges(obstacle, settings)
    return forward > 0 and right < settings.clearance and left > -settings.clearance


def heading_clear(heading, perception, settings):
    """Check that a strip along this heading misses every nearby object."""
    limit = settings.clearance * 0.98
    sin_h, cos_h = math.sin(heading), math.cos(heading)
    for obstacle in perception.obstacles:
        if obstacle.distance > settings.plan_range:
            continue
        forward, right, left = edges(obstacle, settings)
        ahead = forward * cos_h + max(right * sin_h, left * sin_h)
        sides = [-forward * sin_h + y * cos_h for y in (right, left)]
        if ahead > 0 and min(sides) < limit and max(sides) > -limit:
            return False
    # An object without a distance could be anywhere along its direction.
    half = math.atan2(settings.clearance, settings.trigger_distance)
    return not any(low <= heading + half and high >= heading - half
                   for low, high in perception.unknown)


def unknown_ahead(perception, settings):
    """Check if an object without a distance is roughly straight ahead."""
    return not heading_clear(0.0, Perception(0.0, unknown=perception.unknown), settings)


def to_path(pose, forward, side):
    """Convert a point from the robot's frame to the path frame."""
    x, y, yaw = pose
    return (x + forward * math.cos(yaw) - side * math.sin(yaw),
            y + forward * math.sin(yaw) + side * math.cos(yaw))


def lane_clear(lane_y, pose, clear_x, perception, settings):
    """Check that a lane parallel to the path misses everything up to clear_x."""
    limit = settings.clearance * 0.98
    for obstacle in perception.obstacles:
        if obstacle.distance > settings.plan_range:
            continue
        forward, right, left = edges(obstacle, settings)
        points = [to_path(pose, forward, right), to_path(pose, forward, left)]
        if min(x for x, _ in points) > clear_x:
            continue
        if (min(y for _, y in points) - limit < lane_y
                < max(y for _, y in points) + limit):
            return False
    return True


def candidates(pose, perception, settings):
    """List (heading, lane) options: just clear each obstacle, or go through a gap."""
    options = []
    for obstacle in perception.obstacles:
        if obstacle.distance > settings.plan_range:
            continue
        forward, right, left = edges(obstacle, settings)
        for y, sign in ((left, 1), (right, -1)):
            reach = math.hypot(forward, y)
            if reach > settings.clearance:
                heading = math.atan2(y, forward) + sign * math.asin(settings.clearance / reach)
                lane = to_path(pose, forward, y)[1] + sign * settings.clearance
                options.append((heading, lane))
    for gap in perception.gaps:
        aim = clamp(-pose[2], gap.center_min, gap.center_max)
        side = gap.distance_z * math.tan(aim)
        forward = gap.distance_z + settings.camera_forward
        options.append((math.atan2(side, forward), to_path(pose, forward, side)[1]))
    return options


def choose_plan(pose, perception, blockers, settings):
    """Pick the safe lane closest to the path, or None if there is no safe lane."""
    yaw = pose[2]
    nearest = min(blockers, key=lambda item: edges(item, settings)[0])
    forward, right, left = edges(nearest, settings)
    center = math.atan2((right + left) / 2, forward)
    clear_x = to_path(pose, forward, (right + left) / 2)[0] + settings.pass_length
    plans = []
    for heading, lane in candidates(pose, perception, settings):
        goal_heading = wrap(yaw + heading)
        if (abs(goal_heading) <= settings.max_heading and abs(lane) <= settings.max_lateral
                and heading_clear(heading, perception, settings)
                and lane_clear(lane, pose, clear_x, perception, settings)):
            side = 'left' if heading > center else 'right'
            plans.append(Plan(goal_heading, lane, clear_x, side))
    wanted = [plan for plan in plans if plan.side == settings.preferred_side]
    return min(wanted or plans, key=lambda plan: abs(plan.lane_y), default=None)


class AvoidanceController:
    """Turn perception and odometry into one velocity command per cycle."""

    def __init__(self, settings=None):
        self.settings = settings or Settings()
        self.state = CRUISE
        self.plan = None
        self.in_lane = False
        self.origin = None
        self.started = None
        self.hold_since = None
        self.blocked_since = None
        self.note = ''

    def to_path_frame(self, raw_pose):
        """Convert an odometry pose to the frame set by the start pose."""
        if self.origin is None:
            self.origin = raw_pose
        x0, y0, yaw0 = self.origin
        dx, dy = raw_pose[0] - x0, raw_pose[1] - y0
        return (dx * math.cos(yaw0) + dy * math.sin(yaw0),
                -dx * math.sin(yaw0) + dy * math.cos(yaw0),
                wrap(raw_pose[2] - yaw0))

    def stop(self, note):
        """Return a stop command without changing the state."""
        self.note = note
        return Command(0.0, 0.0, self.state, note)

    def finish(self, note):
        """End the run."""
        self.state = DONE
        return self.stop(note)

    def step(self, now, raw_pose, perception, scan_blocked):
        """Run one cycle. Missing data (None) always stops the robot."""
        if self.state == DONE:
            return self.stop(self.note)
        if raw_pose is None:
            return self.stop('waiting for odometry')
        pose = self.to_path_frame(raw_pose)
        if self.started is None:
            self.started = now
        if now - self.started > self.settings.max_run_time:
            return self.finish('run time limit reached')
        if perception is None or now - perception.received > self.settings.data_timeout:
            return self.stop('waiting for fresh detections')
        if scan_blocked is None:
            return self.stop('waiting for LiDAR')
        command = self.decide(now, pose, perception)
        return self.apply_scan_stop(now, command, scan_blocked)

    def apply_scan_stop(self, now, command, scan_blocked):
        """Stop forward motion if the LiDAR sees something right in front."""
        if not (scan_blocked and command.linear > 0):
            self.blocked_since = None
            return command
        if self.blocked_since is None:
            self.blocked_since = now
        if now - self.blocked_since > self.settings.blocked_timeout:
            return self.finish('blocked for too long')
        return self.stop('LiDAR stop: something is right in front')

    def decide(self, now, pose, perception):
        """Run the current state."""
        settings = self.settings
        x, y, _ = pose
        near = [item for item in perception.obstacles if in_path(item, settings)
                and edges(item, settings)[0] <= settings.trigger_distance]
        # This applies in every state, so the robot can never drive off the field.
        if x >= settings.target_distance:
            return self.finish('reached the target distance')
        if self.state in (CRUISE, RETURN, HOLD) and near:
            return self.start_avoiding(now, pose, perception, near)
        if self.state == HOLD:
            self.state = CRUISE
        if self.state == CRUISE:
            speed = (settings.creep_speed if unknown_ahead(perception, settings)
                     else settings.cruise_speed)
            return self.drive(pose, math.atan2(-y, settings.line_lookahead), speed)
        if self.state == TURN:
            return self.turn(pose)
        if self.state == PASS:
            return self.passing(now, pose, perception, near)
        if abs(y) < settings.line_tolerance:
            self.state = CRUISE
        return self.drive(pose, math.atan2(-y, settings.rejoin_lookahead), settings.pass_speed)

    def start_avoiding(self, now, pose, perception, blockers):
        """Choose a heading once. Stop and keep trying if nothing is safe."""
        plan = choose_plan(pose, perception, blockers, self.settings)
        if plan is None:
            self.state = HOLD
            if self.hold_since is None:
                self.hold_since = now
            if now - self.hold_since > self.settings.blocked_timeout:
                return self.finish('no safe way around the obstacle')
            return self.stop('no safe heading')
        self.plan, self.hold_since, self.in_lane = plan, None, False
        self.state = TURN
        return self.turn(pose)

    def turn(self, pose):
        """Turn on the spot to the chosen heading."""
        error = wrap(self.plan.heading - pose[2])
        if abs(error) < self.settings.heading_tolerance:
            self.state = PASS
            return Command(0.0, 0.0, self.state, 'turn finished')
        limit = self.settings.max_turn_rate
        return Command(0.0, clamp(self.settings.heading_gain * error, -limit, limit), self.state,
                       'passing on the ' + self.plan.side)

    def passing(self, now, pose, perception, near):
        """Hold the heading until the lane is reached, then follow the lane."""
        x, y, yaw = pose
        plan = self.plan
        # Only an object beyond this obstacle can change the plan here.
        new = [item for item in near
               if x + edges(item, self.settings)[0] * math.cos(yaw) > plan.clear_x]
        if new:
            return self.start_avoiding(now, pose, perception, new)
        if x >= plan.clear_x:
            self.state = RETURN
        if math.copysign(1.0, plan.heading) * (y - plan.lane_y) >= 0:
            self.in_lane = True
        heading = (math.atan2(plan.lane_y - y, self.settings.rejoin_lookahead) if self.in_lane
                   else plan.heading)
        return self.drive(pose, heading, self.settings.pass_speed)

    def drive(self, pose, heading, speed):
        """Steer to a heading, and slow down or stop if the error is large."""
        error = wrap(heading - pose[2])
        limit = self.settings.max_turn_rate
        angular = clamp(self.settings.heading_gain * error, -limit, limit)
        linear = speed * math.cos(error) if abs(error) < 0.5 else 0.0
        return Command(linear, angular, self.state)


def front_blocked(points, settings):
    """Count LiDAR points in a box just ahead of the robot (x forward, y left)."""
    hits = sum(1 for x, y in points
               if settings.body_front < x < settings.stop_distance
               and abs(y) < settings.stop_half_width)
    return hits >= settings.min_stop_points


def parse_detections(payload, received):
    """Read a detector's JSON message. Returns None if it is not fresh."""
    if payload.get('schema_version') != 1 or not payload.get('fresh'):
        return None
    return Perception(
        received,
        [Obstacle(item['bearing'], item['angle_min'], item['angle_max'], item['distance_m'])
         for item in payload['blockers']],
        [(item['angle_min'], item['angle_max']) for item in payload['unknown_intervals']])


def parse_gaps(payload):
    """Read polar_hist's JSON gap message. Returns no gaps if it is not fresh."""
    if payload.get('schema_version') != 1 or not payload.get('fresh'):
        return []
    return [GapOption(item['center_min'], item['center_max'], item['planning_distance_m'])
            for item in payload['gaps']]
