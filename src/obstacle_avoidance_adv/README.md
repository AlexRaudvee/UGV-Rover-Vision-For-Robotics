# obstacle_avoidance_adv

LiDAR-first controller for a straight 5–10 m mission and a fixed obstacle-bypass
sequence. Launching never starts a mission. Call the start service explicitly.

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

Only LiDAR, gyro and forward odometry must be available, with a 1.5 s acquisition
and receipt timeout. Brief failures hold motion; five seconds of input loss
latches a fault. Another publisher on the selected command topic also faults.
A blocked corridor waits for clearance instead of assuming it is free.

There is no cube face/size registration, persistent costmap, or static-landmark
localizer. `/static_obstacle_localizer/status` and `/obstacle_anchor/pose` are no
longer used. Mission odometry still accumulates distance and gyro errors: this
change does not provide drift-free localization or a physical success guarantee.
The LiDAR plane can miss obstacles above/below it when depth is unavailable.

All settings are in `config/controller.yaml`. The footprint defaults to 0.26 m
long by 0.30 m wide, plus an 8 cm side margin; include any attachments when changing
these values. In-place turns check measured obstacles against the circumscribed
footprint. Missing rear rays do not prove rear clearance; initial turns need space
around the rover. Reverse movement is used only for the back-up described above (at most 0.30 m).

## Start

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
