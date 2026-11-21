# Lab 2 Session 2: localisation

Provides live AprilTag distances for Vizanti, pan-tilt scanning, marker maps and
basic/weighted localization. Depends on `l2s1_marker_detection`.

## Live marker distances in 2D and 3D

The current detector publishes **`/marker_detector/detections`**, a
`std_msgs/String` JSON message containing family, ID, four pixel corners, image
size, timestamp and `image_geometry`. Native tags also report `hamming` and
`decision_margin`. The live lab tags use `APRILTAG_standard41h12`, IDs **2 and 3**.

The `ranges` node subscribes to these detections and `/camera_info`; it does not
start another detector, control pan/tilt, publish robot velocity, or change the
robot's starting pose. It calls the bundled **AprilTag 3 `estimate_tag_pose`**
routine through the new Python binding. That routine refines candidate poses
using orthogonal iteration and returns the tag translation in camera coordinates.

The detected square measures **14 cm**, even though the full printed pattern is
30 cm. `marker_size_m: 0.14` is therefore used. Using 0.30 would overestimate
distances by approximately 30/14. For rectified images, the camera model uses
**P's 3x3 projection block with zero distortion**, rather than raw K/D. Raw-image
corners are undistorted before passing them to AprilTag pose estimation.

The camera translation is transformed to the robot centre using the existing
camera mount and the most recent commanded pan/tilt angles at image exposure.
The head has no measured angle feedback; these angles are assumed commands.
Initially, the range reader assumes the head is centred. It follows the commands
selected by the driver on `/ugv/pan_tilt_commanded`, rather than the normal
joint-state publisher's repeated zeros. Frames just after abrupt head jumps are
excluded for `head_settle_s`; smooth steps below `head_jump_threshold_deg` remain
usable while tracking. The range reader itself does not move the camera; the
localization launch now also starts the marker tracker described below.

`/odom` is interpolated at the image timestamp, and the existing field alignment
maps observations into field coordinates. Median filtering works on those fixed
field positions, so normal robot movement does not lag a filtered scalar range.
Without matched odometry, camera/robot-relative distance is still available;
observed field placement and connecting lines wait for a matched field pose.

Robustness includes decoder-quality limits, a minimum apparent size, a maximum
reprojection error, a physical range limit, a sliding median, and rejection of
large position outliers. Stale/future image timestamps are rejected. Changing the
field start clears old observations. These checks improve repeatability; small
tags and camera/mount calibration still limit physical accuracy.

The limit permits **one corrected bit**: the live small ID 2 tag regularly decodes
with hamming=1 and a strong decision margin. Two corrected bits remain rejected.
The pinned workspace `usb_cam` overlay corrects the 0.8.1 clock-conversion bug
that made fresh frames appear up to almost a second old; the one-second image
freshness limit is retained. Camera and detector queues keep the latest image
instead of accumulating work behind the rest of the rover's perception stack.

### Field locations and editable settings

`config/marker_map.yaml` is shared with the scanner/localization solver:

```yaml
markers:
  APRILTAG_standard41h12:2: [0.0, -0.75]
  APRILTAG_standard41h12:3: [0.0, 0.75]
```

The example above shows the left/right intersections of the halfway line with the 0.75 m
centre circle. The ID assignment follows the current camera observation:
**ID 3 left, ID 2 right**, facing the centre from the negative-x penalty mark.
The actual editable map may contain other positions and IDs; it is preserved.
Use the coordinates of your actual tags, and set `target_ids` separately for tracking.
Two-coordinate map entries denote ground-reference locations; display icons are
schematic landmarks, not measured tag heights. `observed_position_m` separately
reports the measured tag centre in 3D.

`config/marker_ranges.yaml` contains the tag size, filtering limits, mount
geometry, and path to that shared map. `marker_sizes` allows per-ID sizes, e.g.
`{APRILTAG_standard41h12:2: 0.14}`. `start_rover.sh` passes the editable **source
configuration** directly, and the range node checks both files every second.
Editing size/positions needs no rebuild. Topic changes need a node restart.
By default `only_mapped_markers: true` restricts ranging to configured IDs 2/3;
other raw detections remain visible on `/marker_detector/detections`. This avoids
assigning the 14 cm size to unrelated tags. Set it false only when the sizes of
additional tags are also configured correctly.

### Vizanti output

Both Football Field (2D) and Rover + Depth (3D) subscribe to
**`/marker_ranges/result`**. They display the mapped tags, their IDs, measured
horizontal distances from the robot centre, and connecting lines. The reference
field coordinates never determine the distance value; only the AprilTag pose and
camera mount do. Camera-to-tag slant distance is also available in JSON.

A prefix **`~`** identifies a small tag or an estimate still warming up. The live
tags were only about 9 pixels wide at 640x480, so those readings are displayed as
approximate, with one decimal place. Distances become inactive after one second
without a valid observation. Known landmark icons remain grey and say “not
visible”; old distance lines are hidden. A stopped range stream is also detected
by the browser. The ROS MarkerArray output supports other ROS viewers.

If a tag is decoded but cannot pass the distance checks, its icon says **“range
rejected”**, and the status line gives the actual reason (e.g. decoder quality
or image age). “Not visible” is reserved for tags with no recent detection.
The browser also counts a measurement's age after receipt, so a stopped publisher
cannot keep an already-expiring distance looking fresh.

| Topic | Contents |
|---|---|
| `/marker_detector/detections` | Existing pixel detections, image geometry and decoder quality |
| `/marker_ranges/result` | JSON measurements and shared map references |
| `/marker_ranges/markers` | `visualization_msgs/MarkerArray` reference icons and measured-range labels |

Useful JSON fields include `range_m`, `camera_distance_m`, `range_std_m`,
`observed_position_m`, `known_position_m`, `robot_position_m`, `samples`,
`side_px`, `reprojection_error_px`, `low_resolution`, `fresh`, and `age_s`.
`range_std_m` describes recent scatter; it is not a calibrated accuracy bound.

The daily rover launch starts this automatically (`marker_ranges:=false`
disables it). To start ranges and tracking alongside an existing detector and base driver:

```bash
ros2 launch l2s2_localisation marker_ranges.launch.py \
  marker_range_config:=/home/ws/ugv_ws/src/l2s2_localisation/config/marker_ranges.yaml
```

Stop a manually started standalone launch before starting a new daily rover
session. The temporary helper installed during the live rollout is recorded in
`/tmp/ugv_marker_ranges_4.pid` and is stopped automatically by `start_rover.sh`
before the daily launch takes over. Camera/detector recovery helpers are handed
over the same way, using PID and executable checks. They do not reset odometry
or the field start pose.

```bash
ros2 topic echo /marker_ranges/result --once --full-length
```

After deploying dashboard changes, hard-refresh Vizanti with Ctrl+Shift+R.

## Smooth camera tracking during localization

`marker_ranges.launch.py` starts `marker_pan_tracker` by default. The daily rover
launch includes it, and stopping that localization launch stops tracking.
`pan_tracking:=false` disables just tracking; `marker_ranges:=false` in the daily
launch disables both ranges and tracking. Detection alone does not move the head.
Disable tracking before using the separate scanner or manually controlling the head.

The tracker accepts a list of allowed tag IDs, initially all six field markers
**0, 1, 2, 3, 4, 5**. Any two or more visible targets can drive tracking; the
other configured tags need not be visible. It converts image positions to angular
bearings with the matching raw/rectified calibration, compensates for pan at
image exposure, and filters each tag separately. It pans toward the midpoint of
the leftmost and rightmost retained bearings to center the entire group. This
avoids a cluster of tags on one side pulling the camera away from the farthest
tag on the other side. A deadband suppresses small changes; the pan setpoint
advances at most 8 degrees/second at 20 Hz. Tilt stays at the configured angle.

A target survives two consecutive missing frames and is discarded on the third.
Reappearance resets its counter. Tracking continues while at least two targets
are retained. Losing one of three or more does not force a return to zero.
With fewer than two, the camera smoothly returns to pan 0, even if one
tag remains visible. The configurable observation timeout also handles a stopped
detector. Stale/duplicate frames do not refresh tracks, and decoder quality
checks reject weak IDs. All tags cannot stay visible when they are farther
apart than the camera's view or outside its pan limits.

Edit **`config/pan_tracking.yaml`** and restart localization:

| Parameter | Default | Purpose |
|---|---|---|
| `target_ids` | `[0, 1, 2, 3, 4, 5]` | Allowed physical tag IDs; any two or more can drive tracking |
| `missing_frames` | `3` | Consecutive missed frames before losing a target |
| `detection_timeout_s` | `1` | Maximum time without a usable observation |
| `max_speed_deg_s` | `8.0` | Pan setpoint rate, tracking and returning |
| `deadband_deg` | `3.0` | Ignore small angular errors |
| `smoothing_alpha` | `0.25` | Smaller values smooth more, with more delay |
| `gain` | `0.7` | Fraction of filtered error used for each target |
| `max_pan_deg` | `120.0` | Limit either side of forward |
| `tilt_deg` | `0.0` | Fixed tilt; positive looks up |
| `horizontal_fov_deg` | `0.0` | Zero uses calibration; positive overrides it |
| `fov_range_m` | `4.0` | Sector drawing length, not a detection range limit |

`start_rover.sh` supplies this source YAML, so later parameter edits need a
restart but no build. Range sizes and marker map settings still reload live.
The initial deployment requires rebuilding `l2s2_localisation`, `ugv_bringup`,
`ugv_description`, and `vizanti_server`; `bash start_rover.sh --build` includes
all four. To run a full session with ranging but no tracking:

```bash
bash start_rover.sh pan_tracking:=false
```

The driver gives `/localization/pan_tilt_command` priority over normal joint
commands while the tracker publishes. `/ugv/pan_tilt_commanded` reports selected
commands to ranging and the physical rover's robot state publisher. A one-second
lease restores normal control if tracking stops. The tracker waits for the
driver's selected angle before taking over, so it can begin from an already
turned position. The speed limit applies while tracking is active; the ordinary
controller resumes after it stops. This is commanded position, not measured
encoder feedback or a mechanical zero calibration.

Both Vizanti **Football Field (2D)** and **Rover + Depth (3D)** show a translucent
blue horizontal viewing sector anchored to the robot. It turns with robot yaw
and camera pan. The nominal rectified horizontal FOV is about **117 degrees**,
derived from `ugv_vision/config/camera_info.yaml`'s projection matrix; valid live
CameraInfo updates take precedence. Override `horizontal_fov_deg` if the usable
image is narrower. The sector is drawn on the floor; it is not a full 3D frustum
or a visibility guarantee. Occlusion, black rectification borders, tilt, and tag
resolution can reduce usable coverage. The display is marked estimated because
there is no measured servo feedback. It disappears when tracker state or field
odometry becomes stale. Hard-refresh Vizanti after rebuilding; no extra widget
is required.

```bash
ros2 topic echo /marker_pan_tracker/state --once
ros2 topic echo /ugv/pan_tilt_commanded --once
```

Tracker state includes `tracked_ids` (including tags briefly retained through
dropouts) and `mode: tracking_markers` or `centering`. Vizanti displays the number
of retained tags while tracking. Extend `target_ids` when adding another field
marker; its known coordinates remain configured separately in `marker_map.yaml`.

## Live triangulation beside odometry

The localization launch now starts `marker_triangulation` automatically.
It estimates map x/y from any **two or more known tags in the same image**,
using the existing AprilTag distances and `config/marker_map.yaml`. Two-circle
intersections give two candidate positions. Field bounds and the camera's
pan-corrected bearings reject the mirrored solution. With more tags, range-only
least squares uses all accepted distances. Ambiguous, inconsistent or poorly
constrained observations produce an explicit rejection instead of a guessed fix.

Triangulation receives a new `/marker_ranges/observations` stream containing
measurements **before** odometry-based display filtering. The odometry position
is used only for comparison and is never an input to the position solver.
No odometry fusion, TF correction, robot-start change, or driving is performed.

Both Vizanti views now display **amber ODOM** and **magenta TRIANGULATION**
positions, labels and a comparison line. The status gives the number of tags
used and the separation from odometry at the image timestamp. The ordinary
odometry rover remains at its current position. Rejected or stale triangulation
markers disappear, with the rejection reason in the status.

| Topic | Contents |
|---|---|
| `/marker_ranges/observations` | Same-image native ranges/bearings and known coordinates |
| `/marker_triangulation/result` | JSON fix, candidates, quality and odometry comparison |
| `/marker_triangulation/pose` | New accepted `PoseWithCovarianceStamped` measurements in `map` |

Edit `config/triangulation.yaml` for freshness, range/bearing consistency,
ambiguity and geometry thresholds. Its source file reloads once per second when
started through `start_rover.sh`. Marker positions and field bounds remain in
`config/marker_map.yaml` and also reload live. Configuration changes invalidate
the prior fix until another image arrives. Pose covariance is approximate and
requires validation before future fusion with odometry.

After rebuilding, use `bash start_rover.sh` and hard-refresh Vizanti. To disable
just triangulation, pass `triangulation:=false`. `marker_ranges:=false` disables
the complete live localization group, including triangulation and pan tracking.

```bash
ros2 topic echo /marker_triangulation/result --once --full-length
ros2 topic echo /marker_triangulation/pose --once
```

The derivation, equations, mirror-removal example, solver details, assumptions
and output definitions are in [docs/triangulation.md](docs/triangulation.md),
ready to use as the basis for the lab report.

## Pan-tilt scan and localization

The existing scanner remains available separately:

```bash
ros2 launch l2s2_localisation marker_scan.launch.py marker_size_m:=0.14
```

It assumes the rover stays stationary throughout a scan. Do not start a second
detector over an already-running daily detector. The live distance overlay above
does not require a scan or automatically correct the robot's field pose; that
localization step remains separate.

## Tests

In the sourced ROS container:

```bash
export PYTHONPATH=src/l2s2_localisation:src/l2s1_marker_detection:src/ugv_main/ugv_bringup:$PYTHONPATH
python3 -m pytest -q src/l2s2_localisation/test
python3 -m pytest -q src/l2s1_marker_detection/test
node scripts/test_vizanti_field.cjs
node scripts/test_vizanti_scene3d.cjs
```

The range integration tests use isolated ROS domain **95**, synthetic detections
and temporary configs. They verify that deliberately incorrect map coordinates
cannot change the measured distance, and test staleness, decoder rejection,
camera rectification, scaling, motion compensation and filter reset behavior.
The tracker tests use isolated domain **94** and a fake serial connection, never
the physical head. They cover dropout grace, bounded movement, timeout, exposure
angles, command arbitration, calibration FOV and smooth-motion ranging. Browser
tests cover FOV rotation, placement, expiration and rendering in both views.
Live triangulation integration tests use domain **92** and synthetic camera
corners. They recover position with a turned camera despite deliberately wrong
odometry, operate without odometry, and test stale/insufficient measurements and
configuration changes. Pure solver tests cover mirror ambiguity, noisy ranges,
2–6 markers, collinear layouts and invalid geometry. Browser tests verify both
pose labels, comparison output, rejected-fix hiding and expiration.
