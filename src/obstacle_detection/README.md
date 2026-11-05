# obstacle_detection

ROS 2 Humble black-obstacle detector and polar histogram for the Lab 1 black boxes/rovers. It subscribes to existing camera drivers and publishes perception/debugging output. It has no velocity publisher or motor interface.

## Build and run on this robot

Edit this directory on the host. In the active ROS container:

```bash
docker exec -it ugv_jetson_ros_humble bash
source /opt/ros/humble/setup.bash
cd /home/ws/ugv_ws
colcon build --packages-select obstacle_detection
source install/setup.bash
export ROS_DOMAIN_ID=4
ros2 launch obstacle_detection detection.launch.py camera:=oak
```

The existing camera session (`ugv_vision lab1_view.launch.py`) and Vizanti must already be running. This package does not launch duplicates of those services. The default OAK profile uses its rectified RGB and aligned depth. For the wider USB view, stop the existing detector and run:

```bash
ros2 launch obstacle_detection detection.launch.py camera:=usb
```

To compare both profiles simultaneously, give them different namespaces, e.g. `namespace:=obstacles_usb` and `namespace:=obstacles_oak`. Their scan frames are already distinct. Do not launch a second copy using the same namespace.

## Vizanti display

Open the existing dashboard in Chrome. Click **+ → Compressed Image**, then click its new icon and select:

- `/obstacle_detection/histogram/compressed`: live polar and front-view bar histograms.
- `/obstacle_detection/debug/compressed`: annotated camera image showing the mask, candidate boxes, bearings, and distances.

Add one widget for each topic. Position them side-by-side using the image widget's preview drag control. No TF configuration is required for these image widgets. Socket throttle around 300–500 ms is sufficient.

Move a black box or black rover across the selected camera's field of view. Peaks move with its bearing and grow as the obstacle becomes nearer. Camera-left is positive; camera-right is negative; straight ahead is zero. Rear/unseen bins are gray/unknown. A single fixed camera cannot detect an object behind the robot.

The algorithm detects dark color/shape candidates, not semantic object classes. It can also detect dark clothes, shadows, or unrelated dark objects. The annotated view explains which image region produces each peak. Brightness/saturation filtering helps reject red barriers. Compare results against your own images and tune the baseline for the field.

## Output contract

| Topic under the chosen namespace | Type | Contents |
|---|---|---|
| `polar_histogram` | `sensor_msgs/msg/LaserScan` | Angular bins, candidate radial distances in meters, and scores in `intensities` |
| `histogram_values` | `std_msgs/msg/Float32MultiArray` | Same ordered normalized score bins; NaN means an unobserved direction |
| `detections` | `std_msgs/msg/String` | JSON with freshness, angular grid, observed mask, scores, ranges, bounding boxes, and distance provenance |
| `histogram/compressed` | `sensor_msgs/msg/CompressedImage` | Labeled JPEG histogram for Vizanti |
| `debug/compressed` | `sensor_msgs/msg/CompressedImage` | JPEG camera overlay for Vizanti |

Defaults: 72 bins, 5 degrees each, ordered from -177.5 through +177.5 degrees. Scan `angle_min` and `angle_increment` define the exact angular grid. Score is black-obstacle evidence weighted by proximity, **not** a calibrated collision probability. Zero in a searched bin means no matching black candidate was found; it does not prove the space is safe from every kind of obstacle.

Scan ranges use NaN for unseen/unavailable directions and infinity for searched directions without a retained in-range candidate. Scan scores for unseen bins are zero; use the range NaNs or the JSON `observed` mask to distinguish unknown from searched. `histogram_values` already carries that distinction with NaNs. This is a camera-derived obstacle scan, not a physical LiDAR scan.

The scan's frame is `obstacle_histogram_oak` or `obstacle_histogram_usb`. A static optical-to-forward/left/up rotation attaches it to the input camera frame. Bearings and distances are camera-relative; a pan/tilt camera turning changes their relationship to the robot body. A native Vizanti Laser Scan widget can also display `polar_histogram`; choose the corresponding histogram frame as the Fixed Frame to inspect camera-relative geometry without needing a complete pan/tilt pose estimate.

## Distances and synchronization

OAK depth is used only when RGB/depth timestamps are within `sync_tolerance_s`, dimensions match, and optical frame IDs match. Small input buffers select the latest unprocessed matching pair, allowing depth to arrive after its RGB frame. If no pair arrives within a bounded grace period (at most 0.25 s), processing continues with RGB width estimates. `16UC1` is converted from millimeters; `32FC1` is treated as meters. Invalid/out-of-range depth is excluded; a robust percentile inside each detected contour supplies a range when enough pixels remain. Axial depth is converted to radial horizontal range using the bearing.

If usable depth is missing, or in the USB profile, range is **estimated** from an assumed 0.27 m obstacle width, within the handout's 23–30 cm target range. This approximation changes with actual size, orientation, truncation, and calibration. In the overlay, `D` means measured depth and `~` means a width-based estimate. The JSON `distance_source` is `depth` or `size_estimate`; consumers needing measured distances must check this field. Do not interpret every scan range as a measured depth value.

The whole obstacle silhouette contributes bins, rather than only its center. Scores have a short smoothing/release period to reduce flicker. On missing/stale RGB or missing/invalid calibration, the histogram becomes unknown and the preview explicitly reports the reason. If only depth is missing, RGB detection continues with clearly labeled estimated ranges. This node does not inflate obstacles for robot clearance, choose a gap, or decide robot movement; those belong in the future avoidance controller.

## Tuning

Common settings are in `config/detector.yaml`; sensor inputs are in `config/oak.yaml` and `config/usb.yaml`. Camera/input/grid settings require restart. Detector thresholds, ROI, assumed obstacle width, distance limits, smoothing, and timeouts can be changed at runtime. Examples:

```bash
ros2 param set /obstacle_detection/detector value_max 95.0
ros2 param set /obstacle_detection/detector min_area 250.0
ros2 param set /obstacle_detection/detector object_width_m 0.27
ros2 topic echo /obstacle_detection/detections
```

- `value_max`, `dark_ratio`: upper brightness threshold and adaptation to the bright background.
- `saturation_max`: reject strongly colored objects, including red barriers.
- `min_area`, `min_width`, `min_height`, `min_fill`: reject speckle and thin dark fragments.
- `roi_top`, `roi_bottom`, `side_top`, `side_bottom`: restrict processing and exclude camera/body corners.
- `min_bottom_fraction`: require candidates to reach the lower part of the image; reduces ceiling/window false positives.
- `object_width_m`: size assumption for approximate monocular ranges.
- `stale_after_s`, `sync_tolerance_s`: freshness and RGB/depth pairing limits.
- `smoothing_s`, `release_s`: score smoothing and maximum short persistence after a detection disappears.

Thresholds/areas refer to images resized to at most `max_width` pixels (default 640). Use a custom parameter YAML via `params_file:=...` to retain tuning across runs. Geometry comes from the selected `CameraInfo`; raw USB rays are undistorted using its K/D, while rectified OAK rays use P.

## Implementation and tests

- `perception.py`: black candidates, angular extents, and depth/size range estimation.
- `histogram.py`: angular aggregation, overlap handling, smoothing, and unknown-space semantics.
- `node.py`: ROS callbacks, timestamp pairing, publications, and stale-input behavior.
- `synchronization.py`: buffered RGB/depth selection and bounded fallback waiting.
- `visualization.py`: the two lightweight display images.

In the container after building/sourcing:

```bash
colcon test --packages-select obstacle_detection
colcon test-result --verbose
```

The tests cover calibration/resizing, black versus red/bright objects, bearing sign, depth/fallback provenance, overlapping and moving obstacles, unknown bins, stale input, and actual ROS output. The ROS integration test uses domain 77 and synthetic images; it does not interact with the robot's command topics.
