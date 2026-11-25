# Lab 2 Session 3: curling

Session 3 target-planning code lives here, moved from `l2s2_localisation`.

- `l2s3_curling/astar.py`: grid-based A* search on the inflated obstacle map.
- `l2s3_curling/astar_node.py`: ROS map/pose/goal subscriptions, replanning,
  path publication and Vizanti markers.
- `l2s3_curling/path_plan.py` and `path_node.py`: the original straight-line planner.
- `test/test_path_plan.py`: target coordinates and planner behaviour tests.

The planner displays an obstacle-aware A* path; it does not publish motor commands.
It uses known stand footprints, detected obstacle estimates, the field boundary,
and rover clearance from `/obstacle_map/inflated`. It recalculates every 0.5 s
as the map and pose change and publishes an empty path when the map or pose is
unready/stale, the start or goal is occupied, or no route exists. Diagonal moves
cannot pass through obstacle corners. `/path_planner/state` reports the reason.
For visualization the default `allow_unknown` is true: a route through unknown
space is speculative, not proof of clearance or an autonomous driving command.
The 30 cm footprints, relaxed image timing, detector misses and EKF errors remain
important limits on its accuracy. Measured obstacle placement should be verified
before a controller is attached.
The shared detection, triangulation, pan tracking and EKF remain in the Session 1/2
packages. Existing obstacle avoidance remains in `obstacle_avoidance_adv`; its
intermediate stops need adapting for the curling match's single-stop rule.

## Build and run

On the robot host, `bash start_rover.sh --build-only` includes this package.
Alternatively, inside the sourced ROS container at the workspace root:

```bash
colcon build --packages-select l2s2_localisation l2s3_curling
source install/setup.bash
```

The normal rover session starts the marker EKF, its corrected `map -> odom`
transform, and the planner automatically. To choose a target on the host:

```bash
bash start_rover.sh target_run:=1
```

Choose `target_run` from 1 to 5. The default pose input is `/marker_ekf/pose`;
`/goal_pose` accepts a clicked target. Outputs are `/path_planner/path`,
`/path_planner/markers` and `/path_planner/state`. Disable it with
`path_planner:=false`. Do not start a second planner or marker EKF.

## Slow path follower

The rover launch starts `path_follower` disarmed (`path_follower:=false` disables it).
`/path_follower/start` and `/path_follower/stop` are Trigger services.
When armed it publishes `/cmd_vel` at up to 0.10 m/s and turns at up to 0.25 rad/s.
For heading errors above one radian it uses a slow 0.01 m/s turning arc; this arc
has not been validated on the physical rover and is not swept-volume checked.
After starting with a valid map and path, it continues on the last validated path
for an unlimited time if the obstacle map becomes unavailable, using the last
grid. It also retains the path briefly while the planner reports it is waiting
for the map; when the map has recovered, that path is checked against the current
grid. New obstacles are not detected while the map is unavailable. It stops and disarms when
the pose is stale or the rover strays from the retained path, or if the path, grid
or planner is stale while the map is available,
the goal is reached, it sees another command publisher, localization confidence
is insufficient, or depth timing exceeds the temporary preview limits of 4 s age /
3 s RGB/depth skew. When an obstacle lies more than 0.8 m along the path and the
steering lookahead is clear, it continues for up to 1 s while a replan arrives.
For a nearby blocked path or lookahead, or if the replan doesn't arrive in time,
it sends zero velocity and waits up to 2 s for a fresh, valid path to the same goal; it resumes
automatically only when the checks pass, otherwise it disarms. The motor driver's
existing command timeout remains active. The controller is tested synthetically,
not on the physical rover. Unknown cells are traversable, so an unseen obstacle
may lie on the route. Relaxed depth timestamps and localization errors add further
uncertainty; improve and validate perception and localization before motion.

## Tests

From the workspace root (no ROS required):

```bash
PYTHONPATH=src/l2s3_curling python3 -m pytest -q \
  src/l2s3_curling/test/test_path_plan.py src/l2s3_curling/test/test_obstacle_map.py
```

ROS transport tests additionally require the sourced Humble environment and
`detection_improved`. They use isolated domains and synthetic sensors.

## Obstacle map preview

The normal rover launch starts `obstacle_map` by default (`obstacle_map:=false`
disables it). The A* preview consumes its inflated output; no autonomous
controller or motor output is added.

Only `/detection_improved/object/detections` supplies live obstacle evidence.
There is no raw LiDAR obstacle subscription. Existing LiDAR odometry remains
part of localization. The improved detector retains all measured blockers and
gap estimates; the map does not interpret an apparent gap or an empty detection
list as observed free space.

| Topic | Vizanti / meaning |
|---|---|
| `/obstacle_map/grid` | **Map**: measured floor 0, occupied 100, unknown -1 |
| `/obstacle_map/detected` | **Map**: fixed 30 cm squares for measured detector fronts; no configured stands or boundary |
| `/obstacle_map/inflated` | **Map**: obstacles expanded for rover clearance |
| `/obstacle_map/markers` | **Marker Array**: fixed stand footprints and labels |
| `/obstacle_map/state` | JSON readiness, timing, counts and provisional dimensions |

In Vizanti select fixed frame `map`, add the Map and Marker Array widgets, and
set map opacity around 0.3 so the field remains visible. View either grid or
inflated grid first to distinguish measured obstacles from their clearance area.
These are ROS map overlays; the custom 3D depth widget is unchanged.

Depth evidence is transformed `odom <- camera` at its acquisition timestamp,
stored in continuous odometry coordinates, and reprojected with the latest
EKF-owned `map <- odom`. Known stands remain fixed in map coordinates when the
EKF corrects its position. Applying a saved field start clears live history and
rejects frames captured before the reset.

Only actual measured floor patches become free. The temporary preview settings
retain free observations and measured obstacle fronts for 4 s from capture time.
Both expire to unknown. Fresh floor observations can
clear a previous obstacle in the same cell; an occupied hit wins within a frame.
Missing depth, missing timestamp-matched support masks, stale transforms, an
unusable EKF covariance, or stale alignment metadata withhold free-space output.
The mapper checks input acquisition age (temporarily 4 s), pose/TF age (1 s), position standard
deviation (0.5 m) and heading standard deviation (20 degrees). State explains any
unready condition. These are preview thresholds, not evidence of driving safety.

The configured field perimeter is an occupied navigation boundary, not a detected
wall. Inflation reuses `half_length: 0.125`, `half_width: 0.11` and
`side_margin: 0.06` from `obstacle_avoidance_adv/config/controller.yaml`.
It uses a conservative circle enclosing the rover, plus the margin, voxel size
and twice the largest live blocker range/width standard deviation. These footprint
numbers also need checking against the actual rover before driving. Unknown
space remains unknown outside keepout areas.

The map uses measured blocker fronts rather than all individual above-floor
depth points. It assumes each detected object's footprint is a 30 × 30 cm square
behind the measured front. The orientation is assumed camera-parallel: the
detector does not measure the box's rotation or its back edge. Past observations
can overlap and look larger than one square. An object the detector misses will
not be mapped as occupied, so this preview is not sufficient for autonomous driving.

Edit [config/obstacle_map_preview.yaml](config/obstacle_map_preview.yaml) for map settings and
stand dimensions. **30 × 30 cm is an unmeasured provisional stand/base size**,
centred on each marker-map position; it is not the tag-paper size. Use
`stand_size_overrides` (width, depth), `stand_yaw_deg` (zero: depth along map x),
`measured_stands` and `excluded_stands` for individual markers. Exclude wall tags
without floor obstacles. For example:

```yaml
stand_size_overrides:
  APRILTAG_standard41h12:0: [0.30, 0.20]
measured_stands: ["APRILTAG_standard41h12:0"]
```

`start_rover.sh` passes the editable source map config, marker map, and advanced
avoidance footprint config. Valid edits reload within 1 s and clear live evidence;
invalid edits retain the last valid model, report the error, and withhold readiness.
Frame/topic changes need restarting. This reuses the improved detector's floor
model, all-blocker contract and uncertainty, and the advanced avoidance footprint;
its stop/quarter-turn mission controller is not launched.

To apply source changes, stop the main rover launch yourself, then on the host:

```bash
cd ~/ugv_ws
bash start_rover.sh --build
```

For a separately sourced session, launch only the mapper with
`ros2 launch l2s3_curling obstacle_map.launch.py`; do not duplicate the mapper
already included by the daily launch.
