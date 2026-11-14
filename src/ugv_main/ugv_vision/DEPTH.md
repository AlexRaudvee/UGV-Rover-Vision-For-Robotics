# Shared WLS depth

Both improved dashboard detectors and the Vizanti 3D view now consume the same
metric WLS depth used to produce `/wls_filtered/depth_preview/compressed`.
The topic spelling is `wls_filtered` (WLS = weighted least squares).

```text
OAK metric depth + RGB guide + CameraInfo
                     |
                  WLS filter
                     |
   /wls_filtered/compressedDepth + /wls_filtered/camera_info
          |                   |                    |
    object detector     obstacle detector      depth_cloud -> 3D
          |
   (the preview also renders this same metric depth)
```

The preview is a JPEG colour image, not a metric depth image. Consumers use
`/wls_filtered/compressedDepth`, which contains millimetres encoded as a lossless
16-bit PNG. `/wls_filtered` is the equivalent raw `16UC1` image. The WLS node
publishes `/wls_filtered/camera_info` with intrinsics scaled to its actual output
resolution. Optical frame and original depth capture timestamp are preserved.

## Shared temporal smoothing

The WLS publisher applies a per-pixel EMA **after spatial WLS and before all
metric and preview outputs**. Every consumer of `/wls_filtered` or its
`compressedDepth` transport therefore receives the smoothed depth.

In `config/wls_depth.yaml`, under `wls_filter.ros__parameters`:

```yaml
ema_tau_s: 0.35
ema_max_delta_m: 0.5
```

The new-frame weight is `1 - exp(-capture_interval / ema_tau_s)`. At 10 Hz,
`0.35 s` gives approximately 25% new depth and 75% previous filtered depth.
A larger `ema_tau_s` gives stronger smoothing and more lag; `0` disables EMA.
Pixels changing by more than `ema_max_delta_m` immediately use the new value,
which limits trails when an object enters/leaves a pixel; `0` disables that gate.
Invalid WLS pixels clear immediately. Newly valid pixels initialize directly.
History resets after input outages, capture gaps, clock reversal, resolution or
calibration/frame changes. Original capture timestamps are retained.

This EMA averages image pixels, without compensating camera motion. Some lag
remains for moving objects or a moving rover. Existing per-object range EMAs in
the detector configuration are separate and can add further smoothing.

## Settings you can edit

| File | Settings | Effect |
| --- | --- | --- |
| `config/wls_depth.yaml`, `wls_filter` section | `lambda`, `sigma_color`, `fill_holes` | WLS smoothing, RGB edge sensitivity, interpolation of missing depth. Changes affect both detectors and 3D. |
| Same file | `ema_tau_s`, `ema_max_delta_m` | Shared temporal smoothing and depth-edge reset threshold. |
| Same file | `max_width`, `rate_hz` | Filtered resolution and maximum output frequency. Calibration follows the resolution. |
| Same file, `wls_depth_preview` section | `near_m`, `far_m` | Preview colour scale only. These do not change measured distances. |
| `config/depth_cloud.yaml` | `min_depth`, `max_depth`, `max_points`, `rate` | Visible range in metres and point density/frequency in 3D only. |
| Same file | `max_image_age` | Maximum accepted camera/filter delay. Placement still uses capture-time TF. |
| `config/oak_d_lite.yaml`, `stereo` section | `i_enable_*_filter`, threshold ranges, spatial/temporal parameters | Depth processing on the OAK camera, before WLS. Threshold ranges are millimetres. |
| `../../detection_improved/config/depth.yaml` | `range_min/max`, floor/size/temporal settings | Shared depth detector settings. |
| `../../detection_improved/config/lidar.yaml` | Width/range overrides for the obstacle branch | Loaded after the shared depth configuration even when that branch uses WLS. The filename is retained for compatibility. |

`fill_holes: true` interpolates depth in unsupported pixels. Set it to `false` to
preserve missing measurements. Preview colour limits never recalibrate or rescale
metric distances. Detector temporal filters can smooth a reported distance after
it has been measured from the shared WLS image.

## Apply edits

From the robot host, stop the existing rover session, then:

```bash
cd ~/ugv_ws
bash start_rover.sh --build
```

The build copies configuration into `install`; restarting loads the new settings.
WLS parameters are read-only while its node is running. Keep `wls:=true` (the
default); without WLS the configured consumers correctly wait for data.

For an independent LiDAR obstacle branch again, use
`bash start_rover.sh obstacle_range_source:=lidar`. This changes that branch only;
LiDAR odometry is independent of this setting and continues to use `/scan`.

Inside the sourced ROS container (`ROS_DOMAIN_ID=4`), verify:

```bash
ros2 param get /detection_improved/object/detector depth_topic
ros2 param get /detection_improved/obstacle/detector depth_topic
ros2 param get /depth_cloud depth_topic
ros2 topic hz /wls_filtered/compressedDepth
ros2 topic hz /vizanti/depth/points
```

All three `depth_topic` values should be `/wls_filtered/compressedDepth`.
