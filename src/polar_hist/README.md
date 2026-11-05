# Measured black-object perception

Three ROS 2 Humble packages run independently:

- `object_detection`: calibrated RGB + registered metric depth.
- `obstacle_detection`: calibrated RGB + LiDAR, using TF to project scan returns into the image.
- `polar_hist`: subscribes to both detection streams and produces a separate histogram and gap list for each. The two measurements are not combined or counted twice.

Both detectors find black/near-black connected silhouettes, measure their distance, and calculate physical width. The nominal width band is **0.23 through 0.30 m**, inclusive; a measured uncertainty interval can overlap that band by at most the configurable 1.5 cm tolerance. Borderline detections set `nominal_width: false` and report their uncertainty. No known-width distance estimate is used when range measurements are missing.

## Build and run

From this workspace, in the ROS environment:

```bash
source /opt/ros/humble/setup.bash
python3 scripts/clean_perception_metadata.py install
colcon build --packages-select object_detection obstacle_detection polar_hist ugv_vision ugv_bringup vizanti_server --executor sequential
source install/setup.bash
# Use already-running RGB, depth, LiDAR, and robot TF drivers:
ros2 launch polar_hist perception.launch.py
```

Or build and start the complete existing rover session from the robot host:

```bash
bash start_rover.sh --build
```

The rover launch now starts both detectors and `polar_hist`. `objects:=false`, `obstacles:=false`, and `polar_hist:=false` independently disable them. `obstacle_camera:=usb` selects the calibrated USB camera for RGB/LiDAR; the depth detector still uses OAK RGB. Sensor drivers must be enabled for the chosen inputs.

Individual launches:

```bash
ros2 launch object_detection detection.launch.py
ros2 launch obstacle_detection detection.launch.py camera:=oak
ros2 launch polar_hist histogram.launch.py
```

The nodes publish perception and candidate passages; they do not command motors.

## Input topics and calibration

| Input | Default | Type |
| --- | --- | --- |
| Rectified RGB | `/oak/rgb/image_rect/compressed` | `sensor_msgs/msg/CompressedImage` |
| RGB calibration | `/oak/rgb/camera_info` | `sensor_msgs/msg/CameraInfo` |
| RGB-registered depth (lossless PNG) | `/oak/stereo/image_raw/compressedDepth` | `sensor_msgs/msg/CompressedImage` |
| LiDAR | `/scan` | `sensor_msgs/msg/LaserScan` |

`/oak/rgb/image_rect/compressedDepth` is a transport name advertised by the RGB image publisher; it is not evidence of a depth stream. The depth detector defaults to the lossless compressedDepth PNG transport of the existing stereo depth source, in 16UC1 millimeters or 32FC1 meters. In a live comparison, it delivered 48 frames versus 39 raw frames, with about 190 kB per PNG versus 1.84 MB per raw frame. PNG transport preserves the metric depth values; it improves delivery rather than creating different depth readings. RGB and depth must share the optical frame, resolution, field of view, and timestamps. The OAK config explicitly enables depth alignment to RGB socket 0. The detector rejects mismatched frame IDs or resolutions rather than treating unrelated pixels as registered depth.

For another registered metric depth source, select its topic and transport explicitly. Raw depth remains supported:

```bash
ros2 launch object_detection detection.launch.py \
  depth_topic:=/oak/stereo/image_raw depth_transport:=raw
```

Compressed depth supports ROS PNG transport for 16UC1 and inverse-quantized 32FC1. RVL is rejected. Vizanti receives a normal colorized JPEG, not the metric transport bytes.

The RGB/LiDAR path requires a valid TF chain from the scan frame to the RGB optical frame at the scan timestamp. Translation and rotation are both applied. Scan points must project inside the actual blob contour, including its vertical extent, with at least three supporting returns in a depth cluster. An object above or below the scan plane cannot be ranged by this path. Missing TF, insufficient returns, invalid depth, clipped silhouettes, unsynchronized frames, or stopped input cannot produce accepted detections. Unknown silhouettes remain unavailable to gap selection.

RGB, CameraInfo, and depth use reliable delivery by default because a live rover check received 22 reliable depth frames versus 11 best-effort frames in the same interval. Set `reliable_input: false` for best-effort-only image publishers. LiDAR retains sensor-data QoS. Delayed depth is allowed to match buffered RGB even after the initial arrival grace period; unmatched RGB is not marked consumed. Pairing uses acquisition timestamps, with a default 0.12 s tolerance and a 0.8 s freshness limit checked at both acquisition and receipt. No per-ray scan deskew is performed; fast robot/camera motion requires tighter synchronization and a correct camera pose. The existing pan/tilt pose in robot TF must match the actual camera position.

## Width geometry

For a blob's left/right image edges, normalized optical rays are `qL=(uL-cx)/fx` and `qR=(uR-cx)/fx`. Rectified images use CameraInfo P; raw images use K and distortion correction.

```text
bearing = -atan(qCenter)                       # positive to camera left
width = Z * abs(qR - qL)                       # optical axial depth Z
radial_range = Z / cos(bearing)                # horizontal camera-relative range
width = radial_range*cos(bearing)*abs(tan(edge1)-tan(edge2))
```

Depth samples come from an eroded blob mask. A surface needs enough pixels, at least 20% of valid samples, and a bounded median absolute deviation. This rejects minority near speckles and broad inconsistent depth distributions. At least 4% of the masked region must have valid depth. The selected surface uses a trimmed median and reports robust spread. The depth path applies a time-based exponential moving average (EMA) to the current robust per-blob distance before computing physical width. Its default `depth_ema_tau_s` is 0.35 s; the blend is `alpha = 1 - exp(-dt / tau)`, so a new 5 Hz sample has about 44% weight. LiDAR keeps its existing three-frame median. A large range jump must repeat before it is adopted, then its depth EMA resets rather than blending two surfaces. Missing current measurements stay unknown even if old measurements exist. LiDAR returns are transformed to the camera frame; their optical Z supplies the same width calculation. Width assumes the visible silhouette approximates an object face at the sampled Z. Occlusion, segmentation error, angled surfaces, and calibration error affect the result; validate against measured objects on the rover.

## Histograms and 35 cm gaps

The displayed x-axis runs from **0° to 360°**, with **0° right, 90° forward, 180° left, 270° rear**. Camera yaw is resolved through TF in `plot_frame_id` (default `base_link`); a missing camera pose produces an explicit waiting plot. The bar-chart y-axis and circular radius show **measured distance in meters**, from 0 to `display_range_m` (default **3 m**). Returns beyond that display limit are orange and clipped to the chart border; the detector still measures out to 5 m. Every measured silhouette contributes across its angular extent, including wrong-size black objects retained as blockers. Native ROS LaserScan and JSON bearings retain standard ROS angles, 0 forward and positive left, in their declared camera-facing frames. LaserScan ranges contain meters; intensities and `/values` retain proximity scores for consumers. NaN means unknown; infinity means no measured black silhouette in an observed bin. There is no evidence behind the RGB camera, even though LiDAR may scan 360°.

Passages are bounded by two accepted objects. Their facing edge rays are projected to the nearer object's optical-Z plane, where lateral separation after a two-spread measurement-uncertainty allowance must be at least **0.35 m**. Gap `width_m` is the measured opening; `clearance_m` is its conservative lower estimate used for green qualification. This conservative test avoids a false wide gap caused by diagonal separation between objects at different depths. `edge_distance_m` also reports the actual distance between the two measured edge points. A wrong-size black object or unknown silhouette inside the opening blocks it. FOV boundaries are not fabricated obstacles and do not create green gaps.

The green line beneath the bar chart marks qualifying openings. The pale-green line marks candidate center headings after insetting each side by half the required width. `gap_min_width_m` is the required total footprint width; `side_margin_m` optionally adds clearance on each side. A 35 cm opening has no extra clearance for a 35 cm robot. These are local geometric passage candidates between visible black objects; other colors, hidden objects, floor traversability, the robot's swept path, and longitudinal clearance need a navigation layer and full-obstacle collision checking.

Loss of either detector clears only that detector's green gaps and invalidates its histogram, including when its process stops publishing entirely.

## Vizanti

The rover launch supplies `config/vizanti_layout.json` as the default for a new browser session, with six Compressed Image widgets. Existing browser layouts receive the new managed image widgets once after a hard browser refresh; custom widgets and positions are preserved, and the obsolete obstacle histogram topic is migrated. Settings → Import can still load the complete layout. The views use these topics:

| View | Topic |
| --- | --- |
| Source RGB | `/oak/rgb/image_rect/compressed` |
| Colorized metric depth | `/object_detection/depth/compressed` |
| RGB/depth width annotations | `/object_detection/debug/compressed` |
| RGB/LiDAR width annotations | `/obstacle_detection/debug/compressed` |
| RGB/depth histogram + gaps | `/polar_hist/object/histogram/compressed` |
| RGB/LiDAR histogram + gaps | `/polar_hist/obstacle/histogram/compressed` |

Green boxes in debug images are objects accepted against the nominal 23–30 cm band, including labeled borderline measurements within the bounded tolerance. Orange boxes are wrong-size, unmeasured, or clipped candidates, with widths/distances labeled when available.

Machine-readable outputs:

- `/object_detection/detections`, `/obstacle_detection/detections`: version-1 JSON in `std_msgs/msg/String`, including `fresh`, acquisition `stamp`, optical `frame_id`, `detections`, all measured `blockers`, and `unknown_intervals`. Angles are radians; widths and distances are meters. `width_std_m`, `range_std_m`, `sample_count`, and `nominal_width` expose measurement quality. `blob_count` and `measured_count` help distinguish color segmentation from ranging failures.
- `/polar_hist/{object,obstacle}/scan`: `sensor_msgs/msg/LaserScan` in `polar_hist_object` or `polar_hist_obstacle`, attached to its camera optical frame by a static rotation.
- `/polar_hist/{object,obstacle}/values`: `std_msgs/msg/Float32MultiArray`, ordered from the scan's angle_min with its angle_increment.
- `/polar_hist/{object,obstacle}/gaps`: version-1 JSON with `fresh`, `stamp`, frame, required width, side margin, opening bounds, center heading bounds, measured width, edge distance, and planning distance.

The old `/obstacle_detection/histogram/compressed` and `polar_histogram` outputs have moved to `/polar_hist/obstacle/...`. Update saved Vizanti widgets accordingly.

## Tuning and verification

Edit each package's YAML before launch, or adjust detector color/width thresholds live:

```bash
ros2 param set /object_detection/detector value_max 80.0
ros2 param set /obstacle_detection/detector value_max 80.0
ros2 topic echo /object_detection/detections --once
ros2 topic echo /polar_hist/obstacle/gaps --once
```

Both detector and histogram launches accept `params_file` and `use_sim_time`. Sensor topics, transports, and histogram geometry parameters are startup settings. Color defaults are HSV V ≤ 70, S ≤ 120; small noise is rejected by morphology, configurable area limits, and an angular-width feasibility check: a silhouette that cannot reach the minimum target width even at the maximum sensor range is discarded.

```bash
colcon test --packages-select object_detection obstacle_detection polar_hist
colcon test-result --verbose
```

The synthetic ROS test in `polar_hist/test/test_ros_pipeline.py` uses isolated ROS domain 177, synthetic JPEG RGB, registered depth, LiDAR, and TF. It checks both width estimates, measured gaps, missing TF, preview decoding, independent sensor loss, invalid depth, parameter rejection, and stale clearing without publishing motion commands.

Transport implementation references: [ROS Humble compressedDepth codec](https://github.com/ros-perception/image_transport_plugins/blob/humble/compressed_depth_image_transport/src/codec.cpp) and [DepthAI Humble stereo alignment parameters](https://github.com/luxonis/depthai-ros/blob/humble/depthai_ros_driver/src/param_handlers/stereo_param_handler.cpp).

## Depth quality and the assignment PDF

The assignment PDF (page 5) mentions `/oak/stereo/image_raw` and the optional `depthai_ros_driver rgbd_pcl.launch.py`. That launch creates a point cloud from the same stereo depth; it does not provide an independent, less noisy distance measurement. It also launches a camera driver, so do not run it alongside the existing OAK driver. No extra camera driver or SLAM node is required for these fixes.

The live RGB and depth camera models were checked inside the rover container: both are 1280×720 in `oak_rgb_camera_optical_frame`, with matching projection matrices, and depth is metric 16UC1. The existing stereo stream is the correct input. Passive stereo can have little valid depth on textureless black surfaces; invalid values must remain unknown rather than being replaced with an assumed 27 cm object's distance.

`ugv_vision/config/oak_d_lite.yaml` now enables spatial smoothing with zero hole-filling radius, temporal smoothing with persistence disabled, and a 0.15–5 m threshold filter, alongside the existing 5×5 median filter and subpixel depth. These driver settings take effect when the OAK driver is restarted. Detector-level filtering works immediately when the perception nodes restart. Temporal filtering can lag motion, so the driver settings are deliberately conservative. See [Luxonis depth-filter guidance](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth) and the [driver parameter implementation](https://github.com/luxonis/depthai-ros/blob/humble/depthai_ros_driver/src/param_handlers/stereo_param_handler.cpp).

The missing object detector was a separate installation failure: both 0.0.0 placeholder and 0.2.0 package metadata remained in the install tree. Python resolved the old metadata with no `detector` entry point and raised `StopIteration`. The startup/build helper now removes only that known obsolete placeholder. The test suite launches the actual installed executables in isolated domain 178, in addition to synthetic sensor tests in domain 177.

## Live verification after the delivery fix

A 12-second check received 59 depth-detector updates, all fresh, and decoded 59 object overlays, depth previews, and each histogram preview. PNG compressedDepth delivery and retained delayed RGB/depth matches eliminated synchronization dropouts during that check. Camera spatial, temporal, and threshold filters were confirmed enabled in the running driver.

The pictured boxes measured roughly 31–32 cm wide. Some depth frames therefore report `wrong size` against the original 30 cm upper limit even though the objects are visible and ranged. Inspect the labeled widths, `nominal_width`, and `width_band_m` before changing physical limits. The existing LiDAR configuration currently uses a 32 cm upper limit, which was preserved; the depth configuration remains at 30 cm. These are independent per-detector settings. One observed LiDAR gap had about 38 cm of conservative clearance and was colored green.

## Depth EMA tuning

The EMA follows each silhouette independently, using image overlap and acquisition time. New silhouettes, expired measurements, confirmed range jumps, clock rewinds, and camera-frame changes reset it. Zero, NaN, and missing measurements do not update the EMA or create detections from history. Sensor uncertainty and the difference between raw and filtered range are retained when calculating gap clearance. The depth image preview remains the current measured image; annotations and histogram distances use the filtered object ranges.

```bash
# Default: about 44% new data per frame at 5 Hz.
ros2 param set /object_detection/detector depth_ema_tau_s 0.35
# More smoothing, more response lag:
ros2 param set /object_detection/detector depth_ema_tau_s 0.6
# Disable only depth EMA:
ros2 param set /object_detection/detector depth_ema_tau_s 0.0
```

`/object_detection/detections` includes `depth_ema_tau_s` and `range_samples` with each supported blob's raw distance, filtered distance, and effective `ema_alpha`. Existing `detections` and `blockers` schemas are unchanged. EMA suppresses frame-to-frame jitter; it does not fix a systematic depth bias or a physical width that lies outside the configured nominal band.
