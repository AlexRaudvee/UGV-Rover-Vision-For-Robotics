# marker_localization

Lab 2 (Vision for Autonomous Robots): marker detection now, localization and the
curling match later. Session 1 covers detecting a marker and its category, testing
robustness, and comparing preprocessing improvements.

## Layout

| File | Purpose | Needs ROS |
|---|---|---|
| `marker_localization/detection.py` | Preprocessing, multi-family detection, merging, drawing, presets | no |
| `marker_localization/aruco_compat.py` | Same calls on OpenCV 4.5.4 (rover) and >= 4.7 | no |
| `marker_localization/camera_model.py` | K/D from `camera_info.yaml`, undistortion, mapping points back to raw pixels | no |
| `marker_localization/evaluate.py` | Offline robustness comparison of presets on saved images | no |
| `marker_localization/detector_node.py` | Live detection on the rover camera, capture service | yes |
| `test/test_detection.py` | Synthetic-marker tests (`pytest test`) | no |

Pipeline (assignment Fig. 1b): image -> preprocessing -> detect marker + category.
The category is the OpenCV dictionary (`APRILTAG_36h11`, `6X6_250`, ...). All
corners are reported in the original image's pixels, whatever preprocessing was used.

## Presets

| Preset | What it changes | Targets |
|---|---|---|
| `baseline` | grey image, OpenCV default parameters | reference |
| `clahe` | local contrast equalization | backlight, shadows, glare |
| `sharpen` | unsharp mask | mild defocus / motion blur |
| `upscale` | 2x bicubic enlargement | small, distant markers |
| `tuned` | more threshold windows, smaller min perimeter, looser polygon fit, subpixel corners | size range, blur, skew |
| `undistort` | removes lens distortion with `ugv_vision/config/camera_info.yaml` | markers near the image border |
| `improved` | `clahe` + `upscale` + `tuned`, merged with detections on the raw image | combination |

On synthetic tags (see the tests), `improved` finds every marker `baseline` finds,
plus smaller (12-14 px), more blurred (sigma 4) and lower-contrast ones, at about
10x the runtime with five families. CLAHE on its own lost a blurred tag that
`baseline` found. Trim `families` to what the lab uses to save time.

## On the rover

The rover session must already be running (`bash start_rover.sh`), because the
USB camera publishes `/image_raw`. `start_rover.sh` does not build this package,
so build it once in the container:

```bash
docker exec -it -e ROS_DOMAIN_ID=4 ugv_jetson_ros_humble bash
source /opt/ros/humble/setup.bash
cd /home/ws/ugv_ws
colcon build --packages-select marker_localization
source install/setup.bash
ros2 launch marker_localization marker_detection.launch.py            # preset:=baseline etc.
```

Second container terminal:

```bash
ros2 topic echo /marker_detector/detections          # JSON: family, id, corners, runtime
ros2 param set /marker_detector preset baseline      # switch preset live
ros2 param set /marker_detector capture_label d1.0m_a0deg
ros2 service call /marker_detector/capture std_srvs/srv/Trigger '{}'
```

Vizanti: add a Compressed Image widget for `/marker_detector/annotated/compressed`.
Each capture saves the raw PNG, an annotated JPEG and a line in `captures.jsonl`
with the detections of every preset to `~/ugv_ws/runs/marker_captures/` on the host.

## Pan-tilt scan: range and bearing to every marker (Session 2 input)

`geometry.py` turns a detection into a position relative to the robot. It uses
solvePnP for the marker's 3D position in the camera frame, then rotates by the
commanded pan and tilt and adds the camera mount offsets. `scanner_node.py` sweeps
the pan-tilt through `pan_angles_deg`, waits `settle_s` at each stop, collects
detections for `dwell_s`, and reports one entry per marker:

| Field | Meaning |
|---|---|
| `range_m` | floor distance robot centre -> marker (use this for the circles) |
| `bearing_deg` | direction from the robot's heading, left positive |
| `x_m`, `y_m` | marker in the robot frame (x forward, y left) |
| `height_m` | marker centre height: compare with a tape measure to check size, tilt and mount |
| `range_std_m` | spread over the frames at that stop (noise estimate) |
| `pan_deg`, `seen_at_pans_deg` | stop used (marker nearest the image centre) and all stops that saw it |

Triangulation itself is not implemented here.

```bash
ros2 launch marker_localization marker_scan.launch.py marker_size_m:=0.15
# second terminal, first time only: check the pan direction
ros2 service call /marker_scanner/look_left std_srvs/srv/Trigger '{}'   # must turn LEFT
ros2 service call /marker_scanner/center std_srvs/srv/Trigger '{}'
# if it turned right: add pan_sign:=1.0 in config/scanner.yaml (or ros2 param set)
ros2 service call /marker_scanner/scan std_srvs/srv/Trigger '{}'
ros2 topic echo /marker_scanner/result --once --full-length
```

Results are also saved to `~/ugv_ws/runs/marker_scans/scan_*.json`. The pan-tilt
has no angle feedback, so bearings assume the servo reached the commanded angle.
Stop anything else that publishes `/ugv/joint_states` during a scan.

## Triangulation

`triangulation.py` implements the hand derivation (circle 2 - circle 1 gives
A x + B y = C, then the quadratic in x with alpha, beta, gamma), plus:

- **Small B:** when |A| > |B| the same steps run with x and y swapped, to avoid dividing by a tiny B.
- **Circles that don't meet** (noise): uses x = -beta / (2 alpha), the closest point, and adds a warning.
- **Two solutions:** drops a candidate outside the field (`Field`), then compares the bearings. Only the real position sees the markers at the measured angles to each other.
- **3+ markers:** least squares on the stacked rows A_i x + B_i y = C_i, then Gauss-Newton on the original circles. Collinear markers fall back to the two circles farthest apart.
- **Heading:** estimated from the bearings (circular mean of field angle minus bearing).

Put measured marker positions in `config/marker_map.yaml` and rebuild. The
scanner then logs `Robot at x=..., y=..., heading ...` after every scan. Offline:

```bash
python -m marker_localization.localize runs/marker_scans/scan_*.json --map config/marker_map.yaml
python -m marker_localization.localize ... --no-refine     # linear least squares only
python -m marker_localization.localize ... --no-bearings   # shows the mirror ambiguity
```

## Robustness evaluation (offline, laptop)

```bash
cd src/marker_localization
python -m marker_localization.evaluate ../../runs/marker_captures \
    --expect APRILTAG_36h11:3 --out ../../runs/marker_eval
pytest test
```

`--expect` lists markers present in every image. This gives a hit rate per preset.
The output is `detections.csv` (image x preset) and annotated images per preset.
For three distances x three angles, label captures like `d1.0m_a30deg` so the CSV
can be grouped into a table.
