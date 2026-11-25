# obstacle_avoidance_adv

LiDAR-first controller for the five curling field targets or a straight 5–10 m
mission, with EKF goal guidance and a fixed obstacle-bypass sequence. Launching
never starts a mission. Call the start service explicitly.

## Motion

1. APPROACH the original endpoint, with a 1.3 m/s cruise ceiling. Actual speed is
   capped by measured obstacle clearance, scan age and braking distance.
2. STOP approximately 30 cm before the obstacle, measured from the front bumper.
3. Choose the shorter clear sidestep; prefer right when costs differ by at most
   5 cm. Commit to that side, then TURN_OUT 90 degrees using gyro feedback.
4. SIDESTEP by the observed obstacle edge plus chassis width/margin, at least
   0.5 m and at most 1.2 m. Do not continuously resize the manoeuvre from noise.
5. TURN_FORWARD 90 degrees back to the approach heading.
6. PASS 1 m forward, provided the corridor is clear. A new obstacle starts
   another stop/bypass sequence.
7. REJOIN by driving diagonally to the original endpoint. There is no intermediate
   sideways return to the original line. Stop within 8 cm of the estimated goal.

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

A sidestep blocked for 5 s turns back to the route and re-plans from PASS
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

- `/scan`: primary obstacle/clearance source. Chassis reflections are removed;
  adjacent returns support obstacles, and isolated returns must repeat. The last
  0.3 s of scan observations bridge brief missing rays. Broad unobserved forward
  sectors cause a wait; NaNs are not converted to free space.
- `/imu/estimation`: physical gyro transformed into the base frame. Quiet readings
  estimate bias while idle. Integrated gyro angle closes the 90 degree turns.
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
- `/detection_improved/obstacle/detections`: optional WLS depth obstacle outlines.
  Delayed measurements are compensated using mission pose history. NaN/missing
  depth, no detections, or unavailable camera TF do not block LiDAR navigation.
- `/marker_ekf/pose`: required field-frame pose for goal direction, remaining
  distance and arrival in APPROACH/REJOIN. Wheel/gyro integration still supplies
  continuous coordinates for obstacle history and physical bypass distances.
- `/football_field/state`: required assumed start and alignment revision. The
  starting heading is zero. With `target_run:=1` through `5`, the map endpoint
  is the selected assignment coordinate. With `target_run:=0` (the default),
  it is `(start_x_m + goal_distance, start_y_m)`. Marker corrections never move it.

LiDAR, gyro and forward odometry have a 1.5 s acquisition and receipt timeout.
The EKF and its last timestamp-matched alignment have a 1 s timeout, and field
metadata must have been received within 1.5 s. Brief failures hold motion; five seconds of input loss
latches a fault. Another publisher on the selected command topic also faults.
A blocked corridor waits for clearance instead of assuming it is free.

There is no cube face/size registration or persistent costmap. Global goal
guidance uses the external marker EKF; `/static_obstacle_localizer/status` and
`/obstacle_anchor/pose` are no longer used. Local mission odometry still accumulates
distance and gyro errors, while accepted marker corrections adjust goal guidance.
This does not provide drift-free localization or a physical success guarantee.
The LiDAR plane can miss obstacles above/below it when depth is unavailable.

All settings are in `config/controller.yaml`. The footprint defaults to 0.26 m
long by 0.30 m wide, plus an 8 cm side margin; include any attachments when changing
these values. In-place turns check measured obstacles against the circumscribed
footprint. Missing rear rays do not prove rear clearance; initial turns need space
around the rover. Reverse movement is used only for the back-up described above (at most 0.30 m).

## EKF goal guidance and comparison

The controller uses `/marker_ekf/pose` for goal guidance and adds `ekf` and
`navigation` objects to `/obstacle_avoidance_adv/status`. Configure
`ekf_pose_topic`, `ekf_frame` and `ekf_timeout` in `config/controller.yaml`.

EKF and wheel/gyro poses are aligned at the same acquisition timestamp. This
alignment translates and rotates the fixed map goal into the continuous motion
frame. Existing APPROACH/REJOIN steering and arrival checks therefore use the
EKF-corrected distance and bearing. `navigation.pose_map` advances the matched
EKF pose by subsequent wheel/gyro motion; `goal_map_m` remains fixed, while
`goal_motion_m` changes when localisation corrects. No EKF pose overwrites local
scan history, committed bypass targets, turn headings or manoeuvre progress.

Start rejects missing, malformed, stale or excessively uncertain EKF data and
missing/stale field metadata. Motion waits for the first matched sample. If a newer EKF sample
arrives before its matching wheel callback, the last alignment may be retained
only until its own acquisition/receipt timeout. An alignment reset or changed
assumed start holds motion and requires a new mission; it does not move the goal.

The diagnostic reports field-frame position, heading, acquisition/receipt age
and planar standard deviations. `valid` checks the frame, finite values,
orientation and covariance structure; `fresh` additionally checks both ages.
`within_uncertainty_limits` separately checks `max_position_std_m` (default
0.50 m) and `max_heading_std_deg` (default 20 degrees). Position uncertainty is
the square root of the largest eigenvalue of the x/y covariance block, so
correlated diagonal uncertainty is included. Status reports `sigma_position_m`,
both configured limits, and `uncertainty_reason` when a limit is exceeded.

Excessive uncertainty rejects start or immediately holds motion. Recovery within
five seconds uses the existing clearance wait and acceleration ramp before
resuming; a continuous hold longer than five seconds faults the mission. These
provisional limits allow the current assumed-start EKF uncertainty of 0.30 m /
15 degrees. They are filter confidence limits, not guaranteed error bounds or
evidence of the 8 cm physical arrival accuracy; physical tuning remains necessary.

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
PYTHONPATH=src/obstacle_avoidance_adv:$PYTHONPATH python3 -m pytest -q \
  src/obstacle_avoidance_adv/test/test_ekf_observer.py \
  src/obstacle_avoidance_adv/test/test_ekf_navigation.py \
  src/obstacle_avoidance_adv/test/test_ekf_uncertainty.py \
  src/obstacle_avoidance_adv/test/test_pose_frames.py
```

These checks validate transport, coordinate/time matching, corrected goal
steering and arrival, retained bypass progress, and stale/invalid input handling.
They do not measure real driving accuracy.

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
After source changes, rebuild `obstacle_avoidance_adv` and `vizanti_server`,
restart the controller, and hard-refresh the browser to load the updated assets.

```bash
# Private preview output; no motor commands. Still waits for the start service.
bash start_rover.sh --build avoidance:=true target_run:=1
```

For the existing distance mission, leave `target_run:=0` and set `goal_distance`.
The `goal_distance` parameter only determines the endpoint in that mode.

On the host, stop the old launch and build once:

```bash
cd ~/ugv_ws
bash start_rover.sh --build-only
# Preview: commands go to /obstacle_avoidance_adv/cmd_vel.
bash start_rover.sh avoidance:=true goal_distance:=5.0
# Motor output: remains IDLE until the start service.
bash start_rover.sh avoidance:=true avoidance_drive:=true goal_distance:=5.0
```

In a second terminal:

```bash
docker exec -it -e ROS_DOMAIN_ID=4 -e ROS_LOCALHOST_ONLY=0 ugv_jetson_ros_humble bash
source /opt/ros/humble/setup.bash
source /home/ws/ugv_ws/install/setup.bash
ros2 topic echo /obstacle_avoidance_adv/status --once --full-length
ros2 service call /obstacle_avoidance_adv/start std_srvs/srv/Trigger '{}'
# Stop at any time:
ros2 service call /obstacle_avoidance_adv/stop std_srvs/srv/Trigger '{}'
```

Inspect `readiness`, `command_topic`, `front_clearance_m`, `heading_deg`, and
`route_position_m` in status. `/obstacle_avoidance_adv/planned_path` shows current
intent in `odom`. Readiness does not depend on seeing an obstacle at startup.
An unobserved passage still cannot be traversed just because start succeeded.
The driver watchdog stops after 0.5 s without commands; stopping after host/driver
failure depends on firmware and has not been verified. Initial physical runs need
supervision and an available physical stop.
