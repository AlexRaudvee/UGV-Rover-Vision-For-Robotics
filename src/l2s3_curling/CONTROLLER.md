# Curling controller

LiDAR-first controller for the five curling field targets or a straight 5–10 m
mission, with EKF goal guidance and a fixed obstacle-bypass sequence. Launching
never starts a mission. Call the start service explicitly.

## Motion

1. APPROACH the original endpoint, with a 1.3 m/s cruise ceiling. Actual speed is
   capped by measured obstacle clearance, scan age and braking distance.
2. STOP at the configured bumper clearance (currently 5 cm) before the obstacle.
3. Search both sides in one-degree steps for a clear diagonal that reaches the
   required lateral offset. Add `bypass_angle_margin_deg` (default 5 degrees),
   recheck the resulting corridor and stopping space, and choose the smaller
   angle (prefer right within one degree). TURN_OUT using gyro feedback.
4. SIDESTEP along that diagonal to the observed obstacle edge plus chassis
   width/margin, with lateral offset limited to 0.5–1.0 m. The diagonal is
   rejected if its forward progress would pass the goal. Each new LiDAR scan
   checks whether the rover can turn and safely rejoin the goal route; if clear
   for 0.2 s, the sidestep stops early. The planned distance remains a limit.
5. REPLAN from the new position after the diagonal or an early exit. If the
   turn and next metre toward the goal (or the whole remaining distance) are
   visibly clear for 0.2 s while stopped, enter REJOIN. Otherwise stop and
   choose another avoidance move.
   If no goal-bounded move fits within 1 m of the goal, hold for a fresh scan.
6. REJOIN by steering directly to the fixed goal. Obstacle checks stop at the
   goal footprint; obstacles farther beyond it do not trigger a bypass. Stop
   within 8 cm of the estimated goal.

Within 1 m of the goal, a blocker seen by the current LiDAR scan within 35 cm
of the goal may occupy the target. The rover can end a sidestep when the route
to a point 10 cm before that blocker is clear, even if the full route to the
goal is blocked. It approaches at avoidance speed and latches GOAL_OCCUPIED
after settling at that bumper clearance. DONE still requires the usual 8 cm
goal tolerance.

### Gap between two obstacles

Before stopping, the controller looks for a gap between two obstacles once one
is within 1.5 m on the route. A lateral offset is a gap when a corridor parallel
to the route (rover width plus `gap_margin` each side; 0.26 m for the 22 cm rover
with 2 cm) is clear from a run-up point
in front of the obstacles until 0.5 m beyond them, obstacles lie within 0.6 m on
both sides of that corridor, and the drive to the run-up point is clear. The gap
nearest the route wins, and the rover drives through its middle:

1. GAP: drive to the run-up point, 0.6 m before the nearest obstacle beside the
   gap (the obstacles may be staggered by up to 0.5 m), re-aiming at it every
   cycle. The point is at most 40 degrees and 0.8 m off the route. If the speed
   limit stops the rover within 10 cm of the point, it lines up from there; if
   something blocks the drive for 3 s further out, it stops and sidesteps.
2. GAP_ALIGN: gyro turn on the spot until parallel to the route.
3. GAP_THROUGH: steer for the middle of the gap as the LiDAR measures it (the
   nearest returns on each side), falling back to the planned centre line while a
   side is not seen, until 0.5 m past obstacles up to 0.5 m long. Corrections are
   limited to 8.5 degrees so the rover never turns on the spot inside the gap. In run
   10d the odometry-only line ended 10 cm off centre, with a box corner inside the
   rover's width just ahead; the back-up below then frees it and the centring steers
   it away. The sides of the gap are known to be close, so only something
   directly in the rover's path (its own width) stops it, and it must stay there
   for 3 s before the rover gives up on the gap; until then it waits. This keeps
   noisy LiDAR points from the gap's sides from stopping it. In simulation (22 cm
   rover, 2 cm gap margin, slow turns, 1 cm LiDAR noise) every run passed gaps of
   30 cm and more; 28 cm went through in half the runs, otherwise it sidestepped.
4. REJOIN as above.

A single obstacle, a gap narrower than the corridor, or a gap that is blocked
behind, falls back to the STOP and sidestep sequence. If the gap becomes blocked
while driving through, the rover stops and chooses a sidestep instead. Tests:
`test/test_gap.py` (no ROS needed).

If STOP finds both sidesteps blocked for 5 s, the rover looks around instead of
waiting. Standing still, it checks the 360 degree LiDAR scan every 5 degrees from the
route, nearest the route first and the freer side first (fewer LiDAR returns within
1.2 m): 0, 5 freer, 5 other, 10 freer, ... 180. A direction is clear when nothing is
within 0.8 m ahead of the bumper across the rover plus side margin, and the LiDAR has
observed that strip. It turns to the first clear direction (LOOK), checks again
(LOOK_CHECK), and drives 0.5 m that way (ESCAPE), then REJOIN plans again from the new
view. If no direction is clear, it turns 90 degrees towards the freer side, to see
past anything hiding part of the scan, and checks again. It never gives up, so only
the 600 s mission timeout ends it. The step is `LOOK_STEP_DEG` in `planner.py`.

When driving needs a turn of more than about 11 degrees, the rover turns on the
spot first. If something is inside its turning circle (run 10f: after a gap, in
REJOIN), it turns while creeping forward at 0.08 m/s instead, provided the way
ahead is clear.

Back-up: when something is within 8 cm ahead of the bumper, or inside the turning
circle when a turn on the spot is needed and no arc is possible, the rover waits
1 s, then backs straight up at 0.05 m/s, re-checking 15 cm behind it, until it can
turn again, by at most 0.30 m. Then the state's own logic carries on (STOP plans a
sidestep or looks around). If that does not free it within 10 s, it looks around
instead. It FAULTs only when it has backed up the full 0.30 m with nothing behind and
something is still within 8 cm ahead: that moves with the rover, such as a cable in
front of the LiDAR (runs 10d, 10f and 10g waited for good before these changes).

A sidestep blocked for 5 s stops and enters REPLAN
(run 10b waited 68 s here until the mission timeout).

Turns finish within the configured 0.04 rad tolerance, followed by a short settle;
once inside, settling continues within twice the tolerance, so gyro drift at the
edge of the band cannot restart the settle timer forever.
Forward speed ramps at 0.5 m/s²; turn speed ramps at 1.2 rad/s². Stops for close
obstacles or unavailable inputs are immediate. The braking model uses 0.5 m/s²
and measured scan age plus 0.2 s reaction time. These are command/model settings;
physical braking and slip have not been measured. A short approach need not reach
1.3 m/s because it must already be slowing for the obstacle.

## Inputs

- `/scan`: sole obstacle/clearance source. Chassis reflections are removed;
  adjacent returns support obstacles, and isolated returns must repeat. The last
  0.3 s of scan observations bridge brief missing rays. Broad unobserved forward
  sectors cause a wait; NaNs are not converted to free space.
- `/imu/estimation`: physical gyro transformed into the base frame. Quiet readings
  estimate bias while idle. Integrated gyro angle closes turns.
- `odom_topic` (now `/odom/wheels`): **forward velocity**, integrated along gyro
  heading for mission distance and diagonal steering. Global EKF pose/yaw jumps and
  lateral velocity do not change the mission position. In run 10c on turf, the
  fused `/odom` reported 0.70 m for a 1.1 m drive (about 35 % short, judged from the
  LiDAR returns of static boxes), while `/odom/wheels` reported 1.05 m. With fused
  odometry every planned distance came out about 50 % long, so the rover drove up
  to the boxes beside a gap. Set `odom_topic: /odom` again once the fusion is fixed.
- `/odom/wheels`: zero-motion detection and fallback if fused forward velocity
  disagrees strongly with wheel feedback. Existing wheel scaling is retained;
  no new wheel calibration is introduced.
- `/marker_ekf/pose`: required field-frame pose for goal direction, remaining
  distance and arrival in every active state. Wheel/gyro integration still supplies
  continuous coordinates for obstacle history and physical bypass distances.
- `/football_field/state`: required assumed start and alignment revision. The
  starting heading is zero. With `target_run:=1` through `5`, the map endpoint
  is the selected assignment coordinate. With `target_run:=0` (the default),
  it is `(start_x_m + goal_distance, start_y_m)`. Marker corrections never move it.

LiDAR, gyro and forward odometry have a 1.5 s acquisition and receipt timeout.
The EKF and its last timestamp-matched alignment have a 1 s timeout, and field
metadata must have been received within 1.5 s. Missing or stale LiDAR holds the
rover until scans return. Other input failures hold motion and latch a fault after
five seconds. Another publisher on the selected command topic also faults.
A blocked corridor waits for clearance instead of assuming it is free.

There is no cube face/size registration or persistent costmap. Global goal
guidance uses the external marker EKF; `/static_obstacle_localizer/status` and
`/obstacle_anchor/pose` are no longer used. Local mission odometry still accumulates
distance and gyro errors, while accepted marker corrections adjust goal guidance.
This does not provide drift-free localization or a physical success guarantee.
The LiDAR plane can miss obstacles above or below it.

All settings are in `config/controller.yaml`. The configured footprint is 0.25 m
long by 0.22 m wide, plus a 6 cm side margin; include any attachments when changing
these values. In-place turns check measured obstacles against the circumscribed
footprint. Missing rear rays do not prove rear clearance; initial turns need space
around the rover. Reverse movement is used only for the back-up described above (at most 0.30 m).

## EKF goal guidance and comparison

The controller uses `/marker_ekf/pose` for goal guidance and adds `ekf` and
`navigation` objects to `/curling_controller/status`. Configure
`ekf_pose_topic`, `ekf_frame` and `ekf_timeout` in `config/controller.yaml`.

EKF and wheel/gyro poses are aligned at the same acquisition timestamp. This
alignment translates and rotates the fixed map goal into the continuous motion
frame. APPROACH/REJOIN steering uses the EKF-corrected distance and bearing.
`navigation.pose_map` advances the matched EKF pose by subsequent wheel/gyro
motion; `goal_map_m` remains fixed, while
`goal_motion_m` changes when localisation corrects. No EKF pose overwrites local
scan history, committed bypass targets, turn headings or manoeuvre progress.

Arrival uses the published FUSED x/y position directly, without subsequent local
projection. In every active state, a distance at or below `goal_tolerance_m`
(default 0.08 m) sends zero velocity and latches DONE once the final-approach
relocalization has completed. This check
precedes scan readiness, timestamp matching and all obstacle manoeuvres, including
back-up. The FUSED pose must still be valid, fresh and use the mission's
unchanged field reference. Visible markers and a final
heading are not required: the EKF continues predicting from odometry without tags.
`navigation.distance_to_goal_m` uses this raw FUSED distance; `projected_distance_to_goal_m` separately reports the steering estimate.
An INFO completion log records the previous state, FUSED position, goal and
distance. The status `arrival` object retains this evidence until the next run.

Start rejects missing, malformed or stale EKF data and
missing/stale field metadata. Motion waits for the first matched sample. If a newer EKF sample
arrives before its matching wheel callback, the last alignment may be retained
only until its own acquisition/receipt timeout. An alignment reset or changed
assumed start holds motion and requires a new mission; it does not move the goal.

The diagnostic reports field-frame position, heading, acquisition/receipt age
and planar standard deviations. `valid` checks the frame, finite values,
orientation and covariance structure; `fresh` additionally checks both ages.
`within_uncertainty_limits` separately checks `max_position_std_m` (default
0.80 m) and `max_heading_std_deg` (default 30 degrees). Position uncertainty is
the square root of the largest eigenvalue of the x/y covariance block, so
correlated diagonal uncertainty is included. Status reports `sigma_position_m`,
both configured limits, and `uncertainty_reason` when a limit is exceeded.

The warning thresholds are advisory in Vizanti. A fresh, valid pose can guide
motion and qualify for arrival even above either warning threshold. The arrival
record and completion log include the EKF uncertainty at the stop. The 8 cm
goal radius applies to the estimated position.

Once per mission, the rover enters `RELOCALIZE` within 1.0 m of the FUSED goal
from APPROACH or REJOIN, after any committed obstacle manoeuvre. The chassis
stops for the full eight seconds while the pan camera holds on visible markers
and searches toward other mapped markers. The search completes before declaring
arrival, including when a correction places the rover inside the goal radius.
The EKF continues to accept observations throughout.
The camera pans at up to 12 degrees/second during this search; normal tracking
uses 8 degrees/second.

After the search, the controller declares arrival only if the latest fresh
FUSED pose remains within tolerance; otherwise it resumes navigation and can
finish when it reaches the goal again without another search. If required inputs
are unavailable it holds. The launch log
records camera actions, accepted marker IDs and before/after pose and uncertainty.

For `ekf.comparison`, the controller captures the assumed start supplied by
`/football_field/state` when its existing start service is called. A zero starting
heading is required for this comparison. The EKF pose is translated into that
start-relative frame and compared with interpolated wheel/gyro history at the
EKF acquisition time. Position and heading differences are EKF minus wheel/gyro.
This assumes the physical mission began at the configured start and heading.
Comparison is withheld while idle, without matching history, after an alignment
change, or when pose/field metadata is stale. Its `reason` explains why.

Synthetic tests require a sourced ROS environment and run on isolated domain
109 with a private command topic. They do not start hardware or a driving mission:

```bash
PYTHONPATH=src/l2s3_curling:$PYTHONPATH python3 -m pytest -q \
  src/l2s3_curling/test/test_ekf_observer.py \
  src/l2s3_curling/test/test_ekf_navigation.py \
  src/l2s3_curling/test/test_ekf_uncertainty.py \
  src/l2s3_curling/test/test_fused_arrival.py \
  src/l2s3_curling/test/test_pose_frames.py
```

These checks validate transport, coordinate/time matching, corrected goal
steering and arrival, retained bypass progress, and stale/invalid input handling.
They do not measure real driving accuracy.

## Avoidance decision snapshots

After starting a mission, obstacle-related stops, reverse attempts, sidestep
selections and gap choices are saved under `run_logs/avoidance_decisions/`.
Each mission gets a timestamped directory with a `decisions.jsonl` timeline.
The launch log gives the path of each snapshot. Each event directory contains
`decision.json` and any matching OAK-D color frame. Images are diagnostic
snapshots only; obstacle decisions use LiDAR points exclusively. Image timestamps
and ages are recorded.
A missing image does not prevent the event log.

The record includes the state change, measured clearance and applied threshold,
the relevant points in rover coordinates and their source and acquisition age.
For sidesteps it includes both options and for a selected gap it includes the
candidate offsets, supporting points on both sides and the chosen entry.
Set `decision_log_dir` to an empty string to turn off recording.

## Vizanti controller overlays

The Football Field and 3D views include a prominent controller state panel.
It shows the selected run, remaining goal distance, current reason, and the
timestamp-matched LOCAL/EKF position and heading disagreement. Input holds,
clearance waits, DONE and FAULT are distinguished; stale status is explicit.

The diagnostic geometry comes from `/curling_controller/visualization` at
10 Hz, while the verbose `/status` topic remains at 2 Hz:

- **Cyan LOCAL circle and heading arrow:** wheel/gyro pose placed on the field
  using the frozen assumed start. It appears after mission initialization; EKF
  corrections never reposition its origin. A dashed cyan line connects it to
  the contemporaneous corrected pose. The numeric disagreement uses acquisition-
  matched samples, not the distance between asynchronous browser updates.
- **FUSED uncertainty:** the panel shows position and heading standard deviations
  while the pose is fresh, and reports unavailable otherwise.
- **Orange action:** an arrow for at most two seconds / 0.6 m of the sent drive
  or reverse command, including curved motion; a rotation arrow for an in-place
  turn, labelled with remaining angle; or a stop indicator during a hold/wait.
  Leg distance is calculated for the current state rather than from a stale
  intermediate target. Action coordinates use the current EKF/local alignment.

Frame/session/revision mismatches and stale data suppress affected geometry.
The LOCAL pose is not presented before initialization or as live after its
motion samples expire. These overlays do not change control behaviour.

`/curling_controller/planned_path` now carries the short drive/reverse command
preview in **map**, replacing the old rough odom-frame waypoint chain. Turns,
stops and unavailable geometry publish an empty path. The rotation cue is drawn
only by the custom field views, since an in-place turn is not a translated path.

## Start

Select a curling target with `target_run:=1` through `target_run:=5`:

| Run | Target | Field `(x, y)` metres |
|---|---|---|
| 1 | Penalty mark | `(3.20, 0.00)` |
| 2 | Goal area corner right | `(3.90, 1.10)` |
| 3 | Goal area corner left | `(3.90, -1.10)` |
| 4 | Penalty area corner right | `(2.85, 2.00)` |
| 5 | Penalty area corner left | `(2.85, -2.00)` |

The table is shared with `l2s3_curling.path_plan`. The assumed start initializes
localisation; changing it does not shift a curling target. Selection is read-only
for each controller process, so restart the controller to change runs. Status
includes `navigation.target_run`, `target_name` and `goal_map_m` after start.

The Football Field and 3D widgets in Vizanti automatically draw a yellow duck
at the controller's selected target, with its run/name and field coordinates.
The ring below the duck marks the exact goal position. The controller's `goal`
status field supplies the marker, including an idle preview before the start
service is called; no separate visual planner is required. Missing/stale status
(over 2.5 s), malformed goals and mismatched field frames hide the marker.
After source changes, rebuild `l2s3_curling` and `vizanti_server`,
restart the controller, and hard-refresh the browser to load the updated assets.

```bash
# Private preview output; no motor commands. Still waits for the start service.
bash start_rover.sh --build curling:=true target_run:=1
```

On the host, stop the old launch, build and launch with motor output enabled.
The controller stays idle until its start service is called.

```bash
cd ~/ugv_ws
bash start_rover.sh --build curling:=true curling_drive:=true target_run:=1
```

For a FUSED forward-distance test, explicitly pass `target_run:=0` and set
`goal_distance`. The original forward-distance obstacle avoider is available
separately through [its own launch options](../obstacle_avoidance_adv/README.md).
The older rover launch spelling `avoidance:=true target_run:=1` through `5`
also selects the curling package, using the same new `/curling_controller` services.

In a second terminal:

```bash
docker exec -it -e ROS_DOMAIN_ID=4 -e ROS_LOCALHOST_ONLY=0 ugv_jetson_ros_humble bash
source /opt/ros/humble/setup.bash
source /home/ws/ugv_ws/install/setup.bash
ros2 topic echo /curling_controller/status --once --full-length
ros2 service call /curling_controller/start std_srvs/srv/Trigger '{}'
# Stop at any time:
ros2 service call /curling_controller/stop std_srvs/srv/Trigger '{}'
```

Inspect `readiness`, `command_topic`, `front_clearance_m`, `heading_deg`, and
`route_position_m` in status. `/curling_controller/planned_path` shows the short
motion preview in `map`. Readiness does not depend on seeing an obstacle at startup.
An unobserved passage still cannot be traversed just because start succeeded.
The driver watchdog stops after 0.5 s without commands; stopping after host/driver
failure depends on firmware and has not been verified. Initial physical runs need
supervision and an available physical stop.
