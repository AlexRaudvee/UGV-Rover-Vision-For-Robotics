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
Initially, the node assumes the head is centred. Frames during head settling are
excluded. No camera movement is commanded by this node.

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

### Field locations and editable settings

`config/marker_map.yaml` is shared with the scanner/localization solver:

```yaml
markers:
  APRILTAG_standard41h12:2: [0.0, -0.75]
  APRILTAG_standard41h12:3: [0.0, 0.75]
```

These are the left/right intersections of the halfway line with the 0.75 m
centre circle. The ID assignment follows the current camera observation:
**ID 3 left, ID 2 right**, facing the centre from the negative-x penalty mark.
Swap the entries if the physical IDs are installed the other way round.
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
disables it). To start only the range reader alongside an existing detector:

```bash
ros2 launch l2s2_localisation marker_ranges.launch.py \
  marker_range_config:=/home/ws/ugv_ws/src/l2s2_localisation/config/marker_ranges.yaml
```

Stop a manually started standalone launch before starting a new daily rover
session. The temporary helper installed during the live rollout is recorded in
`/tmp/ugv_marker_ranges_4.pid` and is stopped automatically by `start_rover.sh`
before the daily launch takes over. It sends no hardware commands.

```bash
ros2 topic echo /marker_ranges/result --once --full-length
```

After deploying dashboard changes, hard-refresh Vizanti with Ctrl+Shift+R.

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
export PYTHONPATH=src/l2s2_localisation:src/l2s1_marker_detection:$PYTHONPATH
python3 -m pytest -q src/l2s2_localisation/test
python3 -m pytest -q src/l2s1_marker_detection/test
node scripts/test_vizanti_field.cjs
node scripts/test_vizanti_scene3d.cjs
```

The range integration tests use isolated ROS domain **95**, synthetic detections
and temporary configs. They verify that deliberately incorrect map coordinates
cannot change the measured distance, and test staleness, decoder rejection,
camera rectification, scaling, motion compensation and filter reset behavior.
